# LegoBasispraktikum

Code für den EV3-Roboter (Pybricks MicroPython v2.0).

## Aufbau

| Datei | Wofür |
|---|---|
| `src/main.py` | Start, Liste der Abschnitte und die Hauptschleife |
| `src/sections/` | eine Datei pro Abschnitt – hier wird programmiert |
| `src/robot.py` | Motoren, Sensoren, Ports und Grundfunktionen |
| `src/menu.py` | Auswahlmenü auf dem Display |

Neuer Abschnitt: Datei mit einer Funktion `run(robot)` in `src/sections/` anlegen und in `src/main.py` eintragen.

In den Abschnitten zum Warten und Fahren `robot.wait()`, `robot.straight()` und `robot.turn()` benutzen,
sonst funktioniert der Abbruch mit Enter nicht.

## Tooltips in VS Code

Einmalig im Projektordner ausführen:

```
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

Unter Windows: `.venv\Scripts\pip install -r requirements.txt`

Danach in VS Code `Ctrl+Shift+P` → **Python: Select Interpreter** → `.venv` auswählen.

Die `.venv` dient nur dem Editor. Gestartet wird der Code über **Download and Run** auf dem Brick.
