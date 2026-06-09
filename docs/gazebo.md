# Запуск Gazebo + ArduPilot SITL на Windows

## Крок 1 — Встановити WSL2 (Ubuntu на Windows)

Gazebo працює лише на Linux, тому потрібен WSL2.

Відкрити **PowerShell від імені адміністратора** і виконати:

```powershell
wsl --install -d Ubuntu-24.04
```

Після встановлення перезавантажити комп'ютер. WSL2 відкриється і попросить придумати логін та пароль для Ubuntu — ввести будь-які.

---

## Крок 2 — Встановити залежності ArduPilot

У терміналі Ubuntu виконати:

```bash
sudo apt update && sudo apt install -y git python3 python3-pip python3-venv \
  gcc g++ make cmake ccache pkg-config
```

---

## Крок 3 — Клонувати репозиторій

```bash
cd ~
git clone --branch feature/gazebo_integration \
  https://github.com/tootiredtoo/ardupilot-technari.git ardupilot-build
cd ardupilot-build
```

> Якщо є SSH-ключ доданий до GitHub:
> ```bash
> git clone --branch feature/gazebo_integration \
>   git@github.com:tootiredtoo/ardupilot-technari.git ardupilot-build
> ```

---

## Крок 4 — Запустити скрипт встановлення Gazebo

Цей скрипт сам встановить Gazebo Harmonic, всі залежності і збере плагін:

```bash
cd ~/ardupilot-build
bash Tools/environment_install/install-gazebo-ubuntu.sh
```

Займе 5–15 хвилин. Наприкінці виконати:

```bash
source ~/.bashrc
```

---

## Крок 5 — Зібрати ArduPilot SITL

```bash
cd ~/ardupilot-build
git -c user.email="x@x.com" -c user.name="x" commit --allow-empty -m "init"
python3 waf configure --board sitl
python3 waf copter
```

> Збірка займає ~1–2 хвилини.

---

## Крок 6 — Запустити симуляцію

```bash
cd ~/ardupilot-build
bash Tools/scripts/gazebo_sitl.sh ArduCopter iris_runway
```

Якщо все працює, в терміналі з'явиться:

```
Starting Gazebo headless: .../iris_runway.sdf
Starting ArduPilot SITL: vehicle=ArduCopter frame=gazebo-iris
...
JSON received:
    timestamp
    imu: gyro
    ...
```

Зупинити — **Ctrl+C** (зупиняє і Gazebo, і SITL разом).

---

## Доступні світи

| Команда | Опис |
|---------|------|
| `gazebo_sitl.sh ArduCopter iris_runway` | Квадрокоптер Iris на злітній смузі |
| `gazebo_sitl.sh ArduCopter iris_warehouse` | Квадрокоптер Iris у складі |
| `gazebo_sitl.sh ArduPlane zephyr_runway` | Літак Zephyr на злітній смузі |
| `gazebo_sitl.sh ArduPlane zephyr_parachute` | Літак Zephyr з парашутом |

---

## Можливі проблеми

| Проблема | Рішення |
|----------|---------|
| `gz: command not found` | Скрипт встановлення не завершився — перезапустити крок 4 |
| `JSON control interface set to 127.0.0.1:9002` але немає `JSON received` | Зачекати 10–15 секунд, SITL сам повторює з'єднання |
| Помилка `Permission denied` при клонуванні через SSH | Використовувати HTTPS-варіант з кроку 3 |
| `waf: error: git submodule status returned 1` | Виконати крок ініціалізації git з кроку 5 |
