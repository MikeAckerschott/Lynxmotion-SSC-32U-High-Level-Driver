# Interface beroepsproduct  
**Auteur:** Mike Ackerschott  

📖 Voor de documentatie, zie: [`Beroepsproduct1/docs`](./Beroepsproduct1/docs)  

---

## 📦 Projectstructuur
Dit project bestaat uit twee packages:
- **`msg_srv`** → bevat de custom service- en message-files  
- **`robo_driver_dll`** → het programma zelf  

---

## ⚙️ Builden van het project
Navigeer naar de directory `/Beroepsproduct1` en voer de volgende commando’s uit:

```bash
colcon build --packages-select msg_srv
colcon build --packages-select robo_driver_dll
````

---

## ▶️ Uitvoeren van het project

> ℹ️ Het programma werkt momenteel alleen op **Linux**, vanwege de manier waarop USB-poorten in Windows worden afgehandeld.

1. **Sourcen van de workspace**
   Na het builden, zorg dat je in de `/Beroepsproduct1` directory zit en source de workspace in **twee terminals**:

   ```bash
   . install/setup.bash
   ```

2. **Start de nodes**

   * In de **eerste terminal**:

     ```bash
     ros2 run robo_driver high_level_node
     ```
   * In de **tweede terminal**:

     ```bash
     ros2 run robo_driver cli_communication
     ```

3. **Uitvoeren van commando’s**
   In de tweede terminal (met `cli_communication`) kun je de onderstaande commando’s gebruiken.

---

## ⌨️ Beschikbare commando’s

### `singleServo int servo int degrees int speed|time`

Zet de geselecteerde servo naar de aangegeven hoek, met **snelheid** of **duur**.

**Voorbeelden:**

```bash
singleServoCommand servo:0 angle:45 duration:1000
singleServoCommand servo:0 angle:45 speed:100
```

---

### `multiServo {int servo int degrees int speed|time} ...`

Zet meerdere servos naar de opgegeven hoeken, met snelheid of duur.

**Voorbeeld:**

```bash
multiServo {servo:0 angle:45 duration:4000} {servo:1 angle:60}
```

---

### `stop`

Noodstop: stopt de beweging onmiddellijk en negeert verdere requests.

**Voorbeeld:**

```bash
stop
```

---

### `start`

Haalt het systeem uit de noodstop en accepteert weer inkomende requests.

**Voorbeeld:**

```bash
start
```

---

### `programmedPosition string position`

Verplaatst de robotarm naar een van de voorgeprogrammeerde posities:
`park`, `ready` of `straightup`.

**Voorbeelden:**

```bash
programmedPosition park
programmedPosition ready
programmedPosition straight-up
```

---

### `skip`

Slaat de huidige beweging over en gaat door met de volgende in de queue.

**Voorbeeld:**

```bash
skip
```

---
