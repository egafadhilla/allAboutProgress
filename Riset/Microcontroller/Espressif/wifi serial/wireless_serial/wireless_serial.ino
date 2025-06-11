#include <WiFi.h>

// Ganti dengan SSID dan password WiFi kamu
const char* ssid = "egafadhilla";
const char* password = "egaega123";

WiFiServer telnetServer(25);
WiFiClient telnetClient;

IPAddress local_IP(192, 168, 137, 42);
IPAddress gateway(192, 168, 137, 1);
IPAddress subnet(255, 255, 255, 0);

void setup() {
  Serial.begin(115200);
  delay(1000);

  WiFi.begin(ssid, password);

 if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("Gagal atur IP statis!");
  }

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi terhubung!");
  Serial.print("Alamat IP ESP32: ");
  Serial.println(WiFi.localIP());

  telnetServer.begin();
  telnetServer.setNoDelay(true);
  Serial.println("Telnet server aktif di port 23");

  // Contoh debug awal
  debugPrint("ESP32 siap menerima koneksi telnet.");
}

void loop() {
  // Tangani koneksi telnet baru
  if (telnetServer.hasClient()) {
    if (!telnetClient || !telnetClient.connected()) {
      if (telnetClient) telnetClient.stop();
      telnetClient = telnetServer.available();
      debugPrint("Client Telnet terhubung.");
    } else {
      telnetServer.available().stop(); // Tolak koneksi lebih dari satu
    }
  }

  // Contoh debug berkala
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 1000) {
    lastPrint = millis();
    debugPrint("Waktu sekarang: " + String(millis()));
  }

  // Terima perintah dari PuTTY (jika ada)
  if (telnetClient && telnetClient.connected() && telnetClient.available()) {
    String command = telnetClient.readStringUntil('\n');
    debugPrint("Perintah diterima: " + command);
  }
}

void debugPrint(const String& message) {
  // Tampilkan ke Serial Monitor
  Serial.println(message);
  // Tampilkan juga ke Telnet jika terhubung
  if (telnetClient && telnetClient.connected()) {
    telnetClient.println(message);
  }
}
