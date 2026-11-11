# computer/ — most Robot ↔ ROS 2

Pakiet ROS 2 `robot_bridge`: serwer TCP, do którego łączy się robot (ESP32), bufor komend ruchu,
wykrywanie awarii robota i publikacja danych z czujników. Protokół: [PROTOCOL](PROTOCOL).

```
computer/                                (= pakiet ROS 2 robot_bridge)
├── communication/                       odbiór pakietów od robota: TCPserver + parsery lidaru, VELOCITY/ANGLE, RESPONSE/STATE
├── include/robot_bridge/robot_link.hpp  wysyłanie komend, bufor, watchdog (bez ROS-a)
├── src/robot_link.cpp
├── src/robot_bridge_node.cpp            węzeł ROS 2
├── config/robot_bridge.yaml             parametry
└── launch/robot_bridge.launch.xml
```

Odbiór działa jak wcześniej: `TCPserver::receiveData()` czyta bajt typu i przekazuje resztę do odpowiedniego parsera.
Parsery odrzucają ramki lidaru z błędnym CRC i nieprawdopodobne pary VELOCITY/ANGLE, bo robot przesyła surowe
bajty z UART lidaru i ramki bywają ucięte.

## Jak działa

* Komendy (`move` [m], `rotate` [°]) wychodzą z mocą `power_mode` (duty x/255, min. 100) i dostają **id żądania** (1, 2, 3, …), trafiają do kolejki i są wysyłane do robota
  **pojedynczo**: następna wychodzi dopiero po pakiecie RESPONSE z id poprzedniej.
* RESPONSE `status = 0` → komenda wykonana, `1` → nieudana (reszta bufora jest wtedy odrzucana, bo kolejne ruchy
  zakładały wykonanie poprzedniego). Odpowiedź z nieznanym id (np. spóźniona) jest ignorowana.
* Stan robota (pakiet STATE): przy `PROCESSING`/`CALIBRATING` most czeka z wysłaniem, przy `FAILURE` bufor jest odrzucany.
  Przejście `PROCESSING → READY` bez RESPONSE oznacza zgubioną odpowiedź → komenda od razu nieudana.
* Brak odpowiedzi w czasie `max_move_time` / `max_rotation_time` (limity robota z `robot/main/Config.hpp`) `+ response_margin` → `UNRESPONSIVE`.
* Każda komenda kończy się wynikiem na `/robot_bridge/result` (także te odrzucone z bufora, z powodem `not sent: …`).
* Stan robota:

  | stan | znaczenie |
  |---|---|
  | `WAITING_FOR_ROBOT` | brak połączenia |
  | `IDLE` | połączony, dane płyną |
  | `EXECUTING` | robot wykonuje komendę |
  | `UNRESPONSIVE` | **robot się wywalił / zawiesił**: brak danych przez `idle_timeout` albo brak RESPONSE w czasie |

  Przy awarii bufor jest czyszczony. Po `drop_after` s połączenie jest zamykane i most czeka, aż robot połączy się ponownie.

## Uruchomienie

Skrypt kompiluje pakiet (Ninja) i uruchamia serwer TCP, który czeka na robota.
Komendy wysyła się z drugiego terminala, tym samym skryptem.

**Windows** (potrzebny tylko Docker Desktop; za pierwszym razem pobiera ROS 2 Jazzy, ok. 1 GB, potem kompiluje tylko zmienione pliki):

```bat
computer\scripts\ros.cmd
computer\scripts\ros.cmd move 0.2      # w drugim terminalu
computer\scripts\ros.cmd rotate 90
computer\scripts\ros.cmd status        # albo: result | odom | scan | clear | shell | stop | rebuild
```

**Linux** (ROS 2 Jazzy/Humble w `/opt/ros`; workspace w `~/robot_bridge_ws`, zmiana przez `ROBOT_BRIDGE_WS`):

```bash
chmod +x computer/scripts/ros.sh
./computer/scripts/ros.sh
./computer/scripts/ros.sh move 0.2       # w drugim terminalu
```

Przed startem skrypt sprawdza `robot/main/main.cpp` i ostrzega, gdy robot się nie połączy, czyli gdy
`wifiEnabled = false` albo komputer nie ma adresu IP, pod który łączy się robot.