#include "drivers_api.h"
#include "drivers/cs9711/cs9711.h"

GType fpi_device_cs9711_acl16_get_type (void);

typedef struct
{
  FpiUsbTransfer *transfer;
  FpiUsbTransferCallback callback;
  gpointer data;
  GCancellable *cancellable;
  gboolean drain;
} PendingRead;

static PendingRead *pending;
static gboolean fail_command;
static gboolean completed;
static GError *result_error;
static FpImage *result_image;
static gboolean firmware_busy;

static void
finish_read (GError *error, gboolean image)
{
  PendingRead *read = g_steal_pointer (&pending);
  const guint8 expected[] = { 0xea, 0x01, 0x62, 0xa0, 0x00, 0x00, 0xc3, 0xea };

  g_assert_nonnull (read);
  if (!error)
    {
      if (image)
        {
          for (gsize i = 0; i < read->transfer->length; i++)
            read->transfer->buffer[i] = g_test_rand_int_range (0, 256);
          read->transfer->actual_length = 8192;
        }
      else
        {
          memcpy (read->transfer->buffer, expected, sizeof expected);
          read->transfer->actual_length = sizeof expected;
        }
    }
  read->callback (read->transfer, read->transfer->device, read->data, error);
  fpi_usb_transfer_unref (read->transfer);
  g_clear_object (&read->cancellable);
  g_free (read);
}

static gboolean
finish_automatic_read (gpointer unused)
{
  if (g_cancellable_is_cancelled (pending->cancellable))
    finish_read (g_error_new_literal (G_IO_ERROR, G_IO_ERROR_CANCELLED, "Cancelled"), FALSE);
  else if (pending->drain)
    finish_read (g_error_new_literal (G_USB_DEVICE_ERROR, G_USB_DEVICE_ERROR_TIMED_OUT, "Quiet pipe"), FALSE);
  else
    finish_read (NULL, FALSE);
  return G_SOURCE_REMOVE;
}

void __wrap_fpi_usb_transfer_submit (FpiUsbTransfer *, guint, GCancellable *, FpiUsbTransferCallback, gpointer);
void
__wrap_fpi_usb_transfer_submit (FpiUsbTransfer *transfer, guint timeout,
                                GCancellable *cancellable,
                                FpiUsbTransferCallback callback, gpointer data)
{
  g_assert_null (pending);
  g_assert_nonnull (cancellable);
  pending = g_new0 (PendingRead, 1);
  *pending = (PendingRead) { transfer, callback, data, g_object_ref (cancellable), timeout == 80 };
  if (timeout != 180000)
    g_idle_add (finish_automatic_read, NULL);
}

gboolean __wrap_fpi_usb_transfer_submit_sync (FpiUsbTransfer *, guint, GError **);
gboolean
__wrap_fpi_usb_transfer_submit_sync (FpiUsbTransfer *transfer, guint timeout, GError **error)
{
  /* A cancelled host read leaves the reader's previous command active. */
  if (transfer->buffer[1] == 2)
    firmware_busy = FALSE;
  if (transfer->buffer[1] == 1)
    {
      g_assert_false (firmware_busy);
      firmware_busy = TRUE;
    }
  if (fail_command && transfer->buffer[1] == 4)
    {
      g_set_error_literal (error, G_USB_DEVICE_ERROR, G_USB_DEVICE_ERROR_TIMED_OUT, "Command timeout");
      return FALSE;
    }
  return TRUE;
}

static void
fake_open (FpImageDevice *dev)
{
  fpi_image_device_open_complete (dev, NULL);
}

static void
fake_close (FpImageDevice *dev)
{
  fpi_image_device_close_complete (dev, NULL);
}

static void
capture_done (GObject *object, GAsyncResult *result, gpointer unused)
{
  result_image = fp_device_capture_finish (FP_DEVICE (object), result, &result_error);
  completed = TRUE;
}

static void
pump_until (gboolean *condition)
{
  gint64 deadline = g_get_monotonic_time () + 5 * G_TIME_SPAN_SECOND;
  while (!*condition && g_get_monotonic_time () < deadline)
    {
      g_main_context_iteration (NULL, FALSE);
      g_usleep (1000);
    }
  g_assert_true (*condition);
}

static void
start_capture (FpDevice *dev, GCancellable *cancellable, gboolean wait_read)
{
  FpDeviceCs9711 *self = FPI_DEVICE_CS9711 (dev);
  completed = FALSE;
  g_clear_error (&result_error);
  g_clear_object (&result_image);
  fp_device_capture (dev, TRUE, cancellable, capture_done, NULL);
  gint64 deadline = g_get_monotonic_time () + 5 * G_TIME_SPAN_SECOND;
  while (!self->scan_ssm && g_get_monotonic_time () < deadline)
    {
      g_main_context_iteration (NULL, FALSE);
      /* Firmware settling is exercised by the hardware test. */
      if (self->init_ssm && fpi_ssm_get_cur_state (self->init_ssm) == 1)
        {
          fpi_ssm_cancel_delayed_state_change (self->init_ssm);
          fpi_ssm_next_state (self->init_ssm);
        }
      g_usleep (1000);
    }
  g_assert_nonnull (self->scan_ssm);
  /* Skip the reader's settling delay; USB operations are simulated. */
  if (wait_read)
    {
      fpi_ssm_cancel_delayed_state_change (self->scan_ssm);
      fpi_ssm_next_state (self->scan_ssm);
    }
}

static void
test_recovery (gconstpointer scenario)
{
  GType type = fpi_device_cs9711_acl16_get_type ();
  g_autoptr(FpDeviceClass) klass = g_type_class_ref (type);
  FpImageDeviceClass *image_class = FP_IMAGE_DEVICE_CLASS (klass);
  klass->type = FP_DEVICE_TYPE_VIRTUAL;
  image_class->img_open = fake_open;
  image_class->img_close = fake_close;
  g_autoptr(FpDevice) dev = g_object_new (type, "fpi-driver-data", (guint64) 1, NULL);
  g_autoptr(GCancellable) cancellable = g_cancellable_new ();
  g_autoptr(GError) error = NULL;
  firmware_busy = TRUE; /* A previous process may have left it waiting. */
  g_assert_true (fp_device_open_sync (dev, NULL, &error));
  g_assert_no_error (error);

  fail_command = g_str_equal (scenario, "command");
  if (g_str_equal (scenario, "init"))
    {
      completed = FALSE;
      g_clear_error (&result_error);
      fp_device_capture (dev, TRUE, cancellable, capture_done, NULL);
      g_cancellable_cancel (cancellable);
    }
  else
    start_capture (dev, cancellable, !g_str_equal (scenario, "delay"));
  if (g_str_equal (scenario, "cancel"))
    {
      g_cancellable_cancel (cancellable);
      while (!FPI_DEVICE_CS9711 (dev)->deactivating)
        g_main_context_iteration (NULL, FALSE);
      /* Deactivation must wait until the outstanding read acknowledges it. */
      g_assert_false (completed);
      g_assert_true (g_cancellable_is_cancelled (pending->cancellable));
      finish_read (g_error_new_literal (G_IO_ERROR, G_IO_ERROR_CANCELLED, "Cancelled"), FALSE);
    }
  else if (g_str_equal (scenario, "delay"))
    g_cancellable_cancel (cancellable);
  else if (g_str_equal (scenario, "timeout"))
    finish_read (g_error_new_literal (G_USB_DEVICE_ERROR, G_USB_DEVICE_ERROR_TIMED_OUT, "Scan timeout"), FALSE);
  else if (g_str_equal (scenario, "disconnect"))
    finish_read (g_error_new_literal (G_USB_DEVICE_ERROR, G_USB_DEVICE_ERROR_NO_DEVICE, "Disconnected"), FALSE);

  pump_until (&completed);
  g_assert_nonnull (result_error);
  g_assert_null (pending);
  g_assert_null (FPI_DEVICE_CS9711 (dev)->scan_ssm);
  g_assert_null (FPI_DEVICE_CS9711 (dev)->io_cancellable);
  g_assert_false (FPI_DEVICE_CS9711 (dev)->deactivating);
  g_assert_cmpint (fpi_device_get_current_action (dev), ==, FPI_DEVICE_ACTION_NONE);

  /* A new operation on the same open reader must succeed after the failure. */
  fail_command = FALSE;
  g_cancellable_reset (cancellable);
  start_capture (dev, cancellable, TRUE);
  finish_read (NULL, TRUE);
  pump_until (&completed);
  g_assert_no_error (result_error);
  g_assert_nonnull (result_image);
  g_assert_cmpint (fp_image_get_width (result_image), ==, 112);
  g_assert_cmpint (fp_image_get_height (result_image), ==, 73);
  g_assert_true (fp_device_close_sync (dev, NULL, &error));
  g_assert_no_error (error);
  g_clear_object (&result_image);
}

int
main (int argc, char **argv)
{
  g_test_init (&argc, &argv, NULL);
  g_test_add_data_func ("/cs9711/recovery/timeout", "timeout", test_recovery);
  g_test_add_data_func ("/cs9711/recovery/cancel", "cancel", test_recovery);
  g_test_add_data_func ("/cs9711/recovery/delay", "delay", test_recovery);
  g_test_add_data_func ("/cs9711/recovery/command", "command", test_recovery);
  g_test_add_data_func ("/cs9711/recovery/disconnect", "disconnect", test_recovery);
  g_test_add_data_func ("/cs9711/recovery/init", "init", test_recovery);
  return g_test_run ();
}
