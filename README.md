# Vision-Based Automatic Emergency Braking (VAEB)
System automatycznego hamowania awaryjnego oparty wyłącznie na obrazie z kamery.

## Opis projektu

VAEB jest modułowym systemem wspomagania kierowcy (ADAS), którego celem jest wykrywanie zagrożeń kolizyjnych na podstawie obrazu z pojedynczej kamery oraz podejmowanie decyzji o hamowaniu awaryjnym.

Projekt testowany był w środowisku symulacyjnym CARLA. System wykorzystuje detekcję i śledzenie obiektów oraz analizę ich ruchu w celu oszacowania prawdopodobieństwa kolizji i aktywacji hamowania awaryjnego.

## Architektura systemu
```mermaid
flowchart TD

    A["Detekcja obiektów<br/>(YOLO11)"]
    B["Śledzenie obiektów<br/>(ByteTrack)"]

    C["Estymacja<br/>odległości"]
    D["Estymacja<br/>prędkości"]

    E["Sprawdzenie<br/>kryterium bliskości"]
    F["Określenie<br/>czasu do kolizji"]
    G["Obliczanie<br/>przyspieszenia względnego"]
    H["Analiza geometrii<br/>kolizji i dynamiki obrazu"]

    I["Określenie<br/>prawdopodobieństwa kolizji"]
    J["Hamowanie<br/>adaptacyjne"]

    A --> B

    B --> C
    B --> D
    B --> E
    B --> H

    C --> F
    D --> F

    D --> G

    E --> I
    F --> I
    G --> I
    H --> I

    I --> J

    J -. sprzężenie zwrotne .-> A
```

## Struktura projektu

Poniżej znajduje się główna struktura plików i folderów w projekcie:
```text
AED
|   detect_danger.ipynb
|   main.py
|   raport.txt
|   README.md
|   requirements.txt
|   summary.csv
|   
+---config
|       consts.py
|       __init__.py
|       
+---env
|       carla.py
|       test.py
|       __init__.py
|       
+---logger
|       setup.py
|       __init__.py
|       
+---logs
|       aeb_critical.log
|       aeb_warning.log
|       
+---tests
|   |   test_runner.py
|   |   __init__.py
|   |   
|   \---scenarios
|           base_scenario.py
|           scenarios.py
|           __init__.py
|           
\---utils
        control.py
        danger.py
        physics.py
        vision.py
        __init__.py
```

## Cytacja
```text
@software{sadowski2025vaeb,
  author = {Sadowski, Marcin},
  title = {Design and implementation of an automatic emergency braking system},
  year = {2025},
  url = {https://github.com/masadows/vaeb}
}
```
