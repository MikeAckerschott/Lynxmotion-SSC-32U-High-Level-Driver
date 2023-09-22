Interface beroepsproduct door Mike Ackerschott voor WoR Worlds 23/24 S1

Dit project bestaat uit 2 packages, namelijk msg_srv (de custom service en message files) en robo_driver (het programma zelf)

<h1> Builden van het project</h1>

* Voer het volgende commando uit in de directory /Beroepsproduct1:

colcon build

* Als dit niet werkt, bouw dan de packages apart:

colcon build --packages-select msg_srv

colcon build --packages-select robo_driver_dll

<h1> Uitvoeren van het project</h1>

<i> Op dit moment is het programma enkel werkend op linux vanwege de manier hoe USB-poorten worden afgehandelt op windows </i>

* Zorg ervoor dat je na het builden in de /Beroepsproduct1 directory de workspace sourced. Open twee terminals en voer het volgende commando uit in beide terminals:
. install/setup.bash

* Op de eerste terminal voer je het volgende commando uit:
ros2 run robo_driver high_level_node

* Op de andere terminal voer je het volgende commando uit:
ros2 run robo_driver cli_communication

* open nu opnieuw de terminal waar je de cli_communication runt en begin met commando's uitvoeren!

<b> Op dit moment zijn de volgende commando's supported:

singleServoCommand int servo int degrees int speed int time

* voorbeeld:
* singleServoCommand servo:0 angle:45 duration:1000
* singleServoCommand servo:0 angle:45 speed:100

multiServoCommand {int servo int degrees int speed int time} {int servo int degrees int speed int time} 

* voorbeeld:

* multiServoCommand {servo:0 angle:45 duration:4000} {servo:1 angle:60}

stop

* voorbeeld:

* stop

programmedPosition string position

* voorbeeld:

* programmedPosition park

* programmedPosition ready

* programmedPosition straight-up </b>

