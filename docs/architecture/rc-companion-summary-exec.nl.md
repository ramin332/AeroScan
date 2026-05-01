# Draadloze Missie-overdracht via Controller — Directiesamenvatting

**Datum:** 2026-05-01
**Status:** Proof-of-concept afgerond. We adviseren deze transportlaag te
**pauzeren** en voor productie te schakelen naar een alternatieve architectuur.

---

## Wat we wilden bereiken

Een gebouwinspectie-missie die in AeroScan is gepland kunnen vliegen **zonder
dat de piloot de controller hoeft te verlaten** — geen SD-kaart-omweg, geen
"download KMZ → loop naar de controller → wissel kaart → herstart Pilot 2 →
importeer" cyclus.

Concreet: een kleine Android-app op de DJI RC Plus 2 controller waarmee de
piloot een missiebestand (KMZ) kiest en dat draadloos doorstuurt naar de
on-drone computer (DJI Manifold 3), die vervolgens de missie vliegt.

## Wat we hebben gebouwd

End-to-end draadloos transport: **controller → drone-radio → drone E-Port →
on-drone computer**. De piloot kiest een bestand in onze app, het bestand
streamt door de lucht, en komt aan bij de drone-side applicatie die we ook
zelf hebben geschreven. We hebben dit vandaag op echte hardware met een echt
missiebestand geverifieerd.

Dit is de eerste keer dat we in onze setup hebben aangetoond dat missiedata
zonder SD-kaart op een DJI-drone kan worden gezet.

## Wat we hebben geleerd

**Het goede:**
- Het draadloze pad werkt. We hebben elke laag van DJI's "MOP" inter-app-kanaal
  bewezen — registratie, het juiste device-slot adresseren, framing,
  integriteitscontrole (MD5), schone disconnect. De drone-side applicatie ziet
  echte bytes vanuit de controller-side applicatie.
- We hebben vier niet-vanzelfsprekende DJI-valkuilen gedocumenteerd die ons
  onderweg tijd hebben gekost (crashes bij app-start, een
  connectie-flikker-gedrag, een verkeerde adressering, en het feit dat de
  piloot na power-up eenmalig de radio moet "primen" met DJI's eigen app).
  Allemaal vastgelegd in `rc-companion-bringup.md` zodat we er niet nog een
  keer voor betalen.

**De blokkade:**
- **De uploadsnelheid is fysiek begrensd op ~5 KB/sec door de radio.** Dit is
  een gedocumenteerde limiet van DJI's OcuSync-uplink op de fysieke laag —
  niet iets dat we in software aan welke kant dan ook kunnen tweaken. De
  snelheid is identiek voor reliable en unreliable modus.

**Wat dat in de praktijk betekent:**

| Grootte missiebestand | Overdrachtstijd via de radio |
|---|---|
| 50 KB | ~10 sec (acceptabel) |
| 500 KB | ~1,5 min (op de grens) |
| 1 MB | ~3 min (slechte UX) |
| 5 MB | ~15 min (onhaalbaar) |

Een typische AeroScan-inspectie-missie voor een gebouw van enige omvang valt
in het multi-MB-bereik zodra de volledige waypoint-set, foto-metadata en het
verplichte `wpmz/`-pakket van DJI erin zitten. **Bij 5 KB/sec zou de piloot
minutenlang op het opstijgpunt zitten wachten per missie.** Dat is slechter
dan het SD-kaart-proces dat we juist proberen te vervangen.

## Waarom we pauzeren

Deze transportlaag is niet het juiste vehikel voor het overzetten van het
volledige missiebestand. Doorinvesteren in deze laag (bestand wegschrijven
naar disk aan drone-zijde, headers valideren, doorkoppelen naar de "Waypoint
V3"-uitvoer-API van de autopilot) levert ons een pad op dat **fundamenteel te
traag** is voor de missies die we daadwerkelijk genereren. Het overige werk
zou solide engineering zijn, maar zou een feature opleveren die de piloot na
oplevering niet zou kiezen om te gebruiken.

## Aanbevolen pivot

De juiste plek om de missie-generatielogica te zetten is **op de drone zelf**,
op de on-drone computer (Manifold 3). In plaats van *een KMZ op de controller
genereren en die over de trage radio sturen*, sturen we een klein commando
("inspecteer de gebouwomtrek op GPS X, Y, Z met deze parameters") en laten we
de on-drone computer **de KMZ lokaal opbouwen** en aan de autopilot
overhandigen. De radio draagt dan kilobytes aan intentie, geen megabytes aan
waypoints.

Deze pivot:
- Omzeilt het OcuSync-uplink-plafond volledig.
- Hergebruikt de draadloze transportlaag die we zojuist hebben bewezen — voor
  command-and-control payloads, waar 5 KB/sec ruim voldoende is.
- Sluit aan bij waar DJI hun architectuur duidelijk voor heeft ontworpen: de
  Manifold heeft de CPU, de opslag en de directe E-Port-USB-link naar de
  autopilot; de controller is bedoeld als dunne piloot-gerichte laag.
- Laat ons de volledige AeroScan-planningsengine behouden; die draait dan op
  de drone in plaats van op een laptop.

## Wat we behouden

- De Android RC-app (`rc-companion/`) staat in de kast, build-baar en
  installeerbaar. Hij heeft nog steeds waarde voor toekomstige features die
  het controller-naar-drone-kanaal nodig hebben voor *kleine* payloads —
  instellingen, start/stop-commando's, status-pulls, telemetrie-tagging.
- De drone-side listener (`rc_probe.c` op de Manifold) blijft eveneens als
  bekend-werkend sjabloon voor elke command-laag-feature.
- Het bring-up-document legt elke valkuil vast, zodat dit transport later
  weer oppakken uren kost in plaats van dagen.

## Beslissing nodig

Bevestig dat we de missie-generatie-engine als primaire architectuur naar de
Manifold 3 verplaatsen, en dat het radiopad uitsluitend wordt gereserveerd
voor controleberichten. De technische proof-of-concept is klaar; dit is een
"waar besteden we de komende twee weken aan"-vraag.
