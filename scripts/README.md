# Scripts Directory

Acest director conține scripturile Python care implementează funcționalitățile cheie ale simulării dronei de polenizare PollieBee. Mai jos este o descriere detaliată a fiecărui script și a funcționalităților sale principale.

## battery_monitor.py

Acest script gestionează monitorizarea bateriei dronei.

Funcții principale:
- `__init__()`: Inițializează nodul ROS și setează parametrii bateriei.
- `update_battery()`: Actualizează nivelul bateriei bazat pe consumul curent.
- `publish_battery_status()`: Publică starea curentă a bateriei.
- `run()`: Rulează bucla principală de monitorizare a bateriei.

## communication_handler.py

Gestionează comunicarea între dronă și stația de bază.

Funcții principale:
- `__init__()`: Inițializează nodul ROS și setează canalele de comunicare.
- `send_message()`: Trimite mesaje criptate către stația de bază.
- `receive_message()`: Primește și decriptează mesajele de la stația de bază.
- `handle_incoming_messages()`: Procesează mesajele primite.

## control_drone.py

Implementează logica de control de bază a dronei.

Funcții principale:
- `__init__()`: Inițializează controlorul dronei.
- `take_off()`: Gestionează decolarea dronei.
- `land()`: Gestionează aterizarea dronei.
- `move_to_position()`: Mișcă drona către o poziție specifică.
- `adjust_for_wind()`: Ajustează mișcarea dronei pentru a compensa vântul.

## data_logging.py

Se ocupă de înregistrarea datelor din timpul misiunii.

Funcții principale:
- `__init__()`: Inițializează sistemul de logging.
- `log_drone_status()`: Înregistrează starea curentă a dronei.
- `log_flower_detection()`: Înregistrează detecțiile de flori.
- `log_pollination_event()`: Înregistrează evenimentele de polenizare.

## drone_controller.py

Oferă un control mai detaliat al dronei, integrând diverse subsisteme.

Funcții principale:
- `__init__()`: Inițializează controlorul complex al dronei.
- `execute_mission()`: Execută misiunea de polenizare.
- `handle_obstacles()`: Gestionează evitarea obstacolelor în timpul zborului.
- `manage_battery()`: Gestionează nivelul bateriei și decide când să se întoarcă la stația de încărcare.

## flower_detection.py

Se concentrează pe detectarea și localizarea florilor.

Funcții principale:
- `__init__()`: Inițializează sistemul de detecție a florilor.
- `process_image()`: Procesează imaginile pentru a detecta florile.
- `localize_flowers()`: Determină pozițiile 3D ale florilor detectate.
- `publish_flower_positions()`: Publică pozițiile florilor detectate.

## main.py

Servește ca punct de intrare principal pentru simulare.

Funcții principale:
- `__init__()`: Inițializează sistemul principal.
- `start_simulation()`: Pornește toate componentele necesare simulării.
- `run()`: Rulează bucla principală a simulării.

## mission_planner.py

Planifică și gestionează misiunile de polenizare la nivel înalt.

Funcții principale:
- `__init__()`: Inițializează planificatorul de misiuni.
- `generate_survey_pattern()`: Creează un model de survol pentru acoperirea câmpului.
- `plan_mission()`: Planifică misiunea de polenizare pe baza detecțiilor de flori.
- `publish_mission_plan()`: Publică planul de misiune.

## obstacle_avoidance.py

Se ocupă de evitarea obstacolelor în timpul zborului.

Funcții principale:
- `__init__()`: Inițializează sistemul de evitare a obstacolelor.
- `detect_obstacles()`: Detectează obstacolele din jurul dronei.
- `adjust_trajectory()`: Ajustează traiectoria dronei pentru a evita obstacolele.

## path_planner.py

Planifică traiectoria dronei între punctele de interes.

Funcții principale:
- `__init__()`: Inițializează planificatorul de traiectorie.
- `plan_path()`: Planifică traiectoria între punctele de interes.
- `publish_path()`: Publică traiectoria planificată.

## pollination_motor.py

Gestionează mecanismul de polenizare al dronei.

Funcții principale:
- `__init__()`: Inițializează sistemul de polenizare.
- `start_pollination()`: Activează mecanismul de polenizare.
- `stop_pollination()`: Dezactivează mecanismul de polenizare.

## pollination_planner.py

Planifică și gestionează procesul de polenizare.

Funcții principale:
- `__init__()`: Inițializează planificatorul de polenizare.
- `plan_pollination()`: Planifică procesul de polenizare pe baza detecțiilor de flori.
- `execute_pollination()`: Execută procesul de polenizare.

## trajectory_follower.py

Se ocupă de urmărirea traiectoriei planificate de către dronă.

Funcții principale:
- `__init__()`: Inițializează sistemul de urmărire a traiectoriei.
- `follow_trajectory()`: Urmărește traiectoria planificată.
- `adjust_for_wind()`: Ajustează traiectoria pentru a compensa vântul.

## visualize_results.py

Vizualizează rezultatele simulării.

Funcții principale:
- `__init__()`: Inițializează sistemul de vizualizare.
- `update_plot()`: Actualizează graficul cu datele curente.
- `run()`: Rulează bucla principală de vizualizare.

## yolo_detection.py

Implementează detecția obiectelor folosind algoritmul YOLO.

Funcții principale:
- `__init__()`: Inițializează sistemul de detecție YOLO.
- `detect_objects()`: Detectează obiectele din imagine.
- `publish_detections()`: Publică detecțiile.

Acest README.md oferă o imagine de ansamblu a funcționalităților și a structurii scripturilor din directorul scripts/. Fiecare script este conceput pentru a rezolva o problemă specifică în cadrul simulării dronei de polenizare, de la controlul dronei și detecția florilor până la planificarea misiunii și vizualizarea rezultatelor.
