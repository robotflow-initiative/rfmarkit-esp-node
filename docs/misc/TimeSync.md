```diff
diff --git a/components/apps/time_sync.c b/components/apps/time_sync.c
index 2290382..0824f1f 100644
--- a/components/apps/time_sync.c
+++ b/components/apps/time_sync.c
@@ -26,7 +26,7 @@ bool esp_wait_sync_time(const char* const posix_tz) {
ESP_LOGI(TAG, "Initializing SNTP");
sntp_setoperatingmode(SNTP_OPMODE_POLL);
sntp_setservername(0, CONFIG_NTP_SERVER_ADDR);
-    sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);
+    sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);
     sntp_set_time_sync_notification_cb(time_sync_notification_cb);
     sntp_init();

@@ -74,6 +74,17 @@ void app_time_sync(void* pvParameters) {
strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
ESP_LOGI(TAG, "The current date/time in Shanghai is: %s", strftime_buf);

+            if (sntp_get_sync_mode() == SNTP_SYNC_MODE_SMOOTH) {
+                struct timeval outdelta;
+                while (sntp_get_sync_status() == SNTP_SYNC_STATUS_IN_PROGRESS) {
+                    adjtime(NULL, &outdelta);
+                    ESP_LOGI(TAG, "Waiting for adjusting time ... outdelta = %li sec: %li ms: %li us",
+                             (long)outdelta.tv_sec,
+                             outdelta.tv_usec / 1000,
+                             outdelta.tv_usec % 1000);
+                    vTaskDelay(500 / portTICK_PERIOD_MS);
+                }
+            }
             /** Set event **/
             ESP_LOGI(TAG, "Setting NTP_SYNCED_BIT");
             xEventGroupSetBits(g_sys_event_group, NTP_SYNCED_BIT);
```
