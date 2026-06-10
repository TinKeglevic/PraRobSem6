# prarob_interact
Repozitorij za seminarski zadatak kolegija "Praktikum robotike". (https://www.fer.unizg.hr/predmet/prarob)

## Upute
Ovaj paket pruža podlogu za početak implementacije seminarskog zadatka iz osnova robotike, namjenjen je za korištenje u ROS2 Jazzy distribuciji.

Implementacija se temelji na AI agentu ROSA namjenjenom za upravljanjem ROS i ROS2 sustavima. (https://github.com/nasa-jpl/rosa)

ROSA agent koristi API poziv za komunikaciju s LLM modelom i generiranje odgovora unesenom tekstu uz dodatne ROS mogućnosti, kao što je pregled dostupnih servisa.

Osim definicije LLM modela i dodatne konfiguracije osnovnog ponašanja agenta, moguće je definirati dodatne alate koje agent može koristiti.

Kako biste koristili agenta, trebate definirati API ključ u izvršnom okruženju.

Ključ će naknadno biti dostupan, a postaviti ga možete u .env datoteci. (pogledajte .env.example)

Prekopirajte ovaj repozitorij u svoj radni prostor:

```
cd ~/ros2_ws/src
git clone https://github.com/larics/prarob_interact.git
```

Instalirajte potrebne pakete koristeći pip:

```
pip3 install dotenv
pip3 install langchain_openai
pip3 install jpl-rosa
```

Nakon toga, pokrenite standardne ros naredbe za pripremu paketa:

```
cd ~/ros2_ws
colcon build --packages-select prarob_interact # Izborno dodajte --merge-install ili --symlink-install
source install/setup.bash
```

Agenta možete pokrenuti naredbom:
```
ros2 run prarob_interact text_interface
```

Kada razvijete novu funkcionalnost sustava, omogućite agentu korištenje tog koda definirajući novi alat u *tools.py*.

Među unarijed spremnim alatima možete naći alate koji pozivaju prazne metode za direktnu i inverznu kinematiku iz *kinematics.py*, ako i neimplementirani alat za filtriranje yolo detekcije.

Na vama je da implementirate alat za filtriranje yolo detekcije kao i metode direktne i inverzne kinematike sukladno parametrima vašeg robota.
