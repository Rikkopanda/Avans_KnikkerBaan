# Avans KnikkerBaan

Dit project bestuurt een knikkerbaan met een ESP32. De ESP32 leest de positie van de bal, rekent met een PID-regelaar uit welke kant de baan op moet kantelen en stuurt daarna een servo aan. Via de seriele poort stuurt de ESP32 live telemetrie terug, zodat je op je laptop kunt meekijken en instellingen kunt aanpassen.

De code staat vooral in:

- `src/main.cpp`: firmware voor de ESP32.
- `tools/serial_plot.py`: Python GUI voor live grafieken en tuning.
- `tools/track.cpp`: OpenCV camera-tracker als alternatief voor de afstandssensor.

## Componenten

- ESP32 DevKit
- PlatformIO, bijvoorbeeld via VS Code of de PlatformIO CLI.
- Servo op GPIO 27.
- Analoge afstandssensor op GPIO 34.
- 4-digit display met DIN op GPIO 23, CLK op GPIO 18 en CS op GPIO 5.
- Python 3 voor de GUI-tools.
- Optioneel: OpenCV en een camera voor vision tracking.

## Firmware Builden En Uploaden

Open dit project in VS Code met PlatformIO, of gebruik de terminal in deze map.

Build:

```bash
pio run -e A
```

Upload naar de ESP32:

```bash
pio run -e A -t upload
```

Seriele monitor openen:

```bash
pio device monitor -e A
```

De firmware gebruikt `115200` baud. Als de uploadpoort niet automatisch gevonden wordt, voeg dan tijdelijk een poort toe in `platformio.ini`, bijvoorbeeld:

```ini
upload_port = /dev/ttyUSB0
monitor_port = /dev/ttyUSB0
```

Op Linux kan de poort ook `/dev/ttyACM0` zijn.

## GUI Starten

Installeer eerst de Python dependencies:

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

Start daarna de live plot:

```bash
python3 tools/serial_plot.py /dev/ttyUSB0 --baud 115200
```

Gebruik eventueel `/dev/ttyACM0` als dat jouw poort is.

In de GUI zie je onder andere:

- afstand, gefilterde afstand en setpoint;
- snelheid, error en integraal;
- PID-output en servohoek;
- spanning van de afstandssensor;
- sliders voor `Kp`, `Ki`, `Kd`, setpoint, servo-neutral, filters en calibratie.

Belangrijke knoppen:

- `AUTO`: laat de regelaar automatisch sturen.
- `MANUAL`: zet servo aansturing in handmatige modus.
- `RESET`: reset de controllerstatus.
- `SENSOR` / `CV`: wisselt tussen afstandssensor en camera-input.
- `ALL`: opent een extra overzicht met alle signalen.

Je kunt ook een los commando sturen en afsluiten:

```bash
python3 tools/serial_plot.py /dev/ttyUSB0 --send "set:16"
```

## Hoe Het Werkt

De firmware draait in drie vaste ritmes:

- De afstandssensor wordt snel gesampled en met een median filter rustiger gemaakt.
- De PID-regelaar draait op 50 Hz, dus elke 20 ms.
- Telemetrie wordt elke 200 ms via serial uitgeprint als regels die beginnen met `Plot,`.

De afstandssensor wordt omgerekend naar centimeters met een curve:

```text
distance = curveA * voltage ^ curveExp
```

Daarna corrigeert de code voor balradius en baancalibratie. De regelaar vergelijkt de gemeten positie met het setpoint. Uit de error, snelheid en integraal komt een gewenste versnelling. Die wordt met een simpel rolling-ball model omgerekend naar een gewenste baanhoek, en daarna naar een servohoek.

Als de bal dicht genoeg bij het setpoint ligt en bijna niet beweegt, activeert de firmware een settle-stand. De servo gaat dan rustig terug naar neutraal om onnodig trillen te voorkomen.

## Seriele Commando's

Commando's hebben meestal de vorm `naam:waarde`. Voorbeelden:

```text
set:16
kp:2.5
ki:0.35
kd:1.2
neutral:84
travel:35
dir:-1
angle:90
manual:on
auto
reset
source:sensor
source:vision
vision:14.2
```

`angle:<waarde>` zet automatisch manual mode aan. Met `auto` gaat de firmware terug naar automatische regeling.

## Camera Tracking

Een losse OpenCV tracker in C++:

```bash
make track
./track --serial /dev/ttyUSB0 --baud 115200 --camera 0 --beam-cm 30
```

In die tracker klik je twee punten op de baan om het meetbereik te definieren. Met de HSV-trackbars stel je in welke kleur bal gevolgd wordt. Sluiten kan met `q` of `Esc`.
