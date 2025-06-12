# Schwarm-Taskallokation

## Ziel der Aufgabe

- Entwicklung eines Systems zur Taskallokation, das:

    - ein Gebiet (2D oder 3D) effizient aufteilt,

    - jedem Roboter eine Teilaufgabe (Gebietsteil) zuweist,

    - dabei flexibel bzgl. Schwarmgröße und Gebietsgrenzen bleibt.

## Annahmen

- Roboterpositionen sind bekannt (z. B. GPS, Motion Capture, oder internes Mapping).
- Roboter haben ähnliche Fähigkeiten (homogener Schwarm).
- Gebiet ist rechteckig (später erweiterbar auf beliebige Formen).
- Erkundung = vollständige Abdeckung