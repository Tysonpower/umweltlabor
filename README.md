# IGHamspirit Umweltlabor
IGHamspirit Umweltlabor - Umweltsensoren gesteuert von einem ESP32 und Grafana Dashboard

# Hardware

# Software
## ESP32
Arduino Sketch herunterladen und entsprechend die eigenen Daten abändern im markierten Bereich

Zum kompelieren und flashen müssen folgende Bibliotheken heruntergeladen werden, dies geht ganz einfach über die Bibliothekverwaltung in der Arduino IDE:

- Dirk Kaar, Peter Lerup - EspSoftwareSerial
- Adafruit - IO, Unified Sensors, BMP085, BME280, CCS811
- Pawel Kolodziejczyk - Nova SDS011
- Nick O'Leary - PubSubClient

## Server
Die Zugangsdaten zum offiziellen Server werden nur auf Anfrage erteil und auch nur für eine begrenzte Anzahl an IGHamspirit Mitgliedern

- MQTT Server
- Influx DB
- Grafana

# Roadmap
- Daten ins APRS Netz speisen von MQTT Server
