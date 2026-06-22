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