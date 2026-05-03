# Draadloze Missie-overdracht via Controller — Directiesamenvatting

**Datum:** 2026-05-03 (vervangt versie van 2026-05-01 — zie "Wat is veranderd" hieronder)
**Status:** Pivot afgerond. We verplaatsen KMZ-bestanden **niet** via de radio. De Smart3D-mesh staat al op de Manifold; we lezen die daar uit, vullen aan met gimbal-richting per waypoint, en sturen de aangepaste KMZ terug via dezelfde bekabelde route die in de dev-workflow al gebruikt wordt.

---

## Wat is veranderd sinds de vorige versie

De versie van 2026-05-01 adviseerde om **de draadloze transportlaag te pauzeren en te kantelen naar "missie genereren op de drone".** Dat advies was gebaseerd op drie aannames die we nu beter begrijpen:

1. We dachten dat het KMZ-bestand vanaf de RC naar de Manifold verplaatst moest worden.
2. We hadden 5 KB/sec via de radio gemeten en als showstopper aangemerkt voor productiebestanden.
3. We behandelden DJI's Smart Auto-Exploration-output als ondoorzichtig.

Een read-only inspectie van het Manifold-bestandssysteem op 2026-05-03 heeft alle drie veranderd:

- De **3D-mesh van Smart Auto-Exploration staat al op de Manifold**, in standaard `.ply`-bestanden onder `/blackbox/the_latest_flight/dji_perception/1/mesh_binary_*.ply`. Per vlucht: ~1 GB verdeeld over ~50 chunks van vertex+normaal-puntwolken. We hoeven niets te verplaatsen; we lezen het ter plaatse.
- De perceptie-data op de Manifold is **dichter dan de gecureerde wolk die DJI in de KMZ stopt** — 4.700 facades geëxtraheerd vs. 1.651 uit dezelfde KMZ-`cloud.ply`.
- Het vluchtplan (waypoints, gimbal-hoeken, capture-commando's) **staat niet op de Manifold** — DJI bewaart het versleuteld in `expl_plan.bin.enc`. Het vluchtplan blijft in de KMZ op de RC, die de piloot via de bestaande USB-MTP-kabel naar de laptop trekt.

De architectuur kantelt dus: **AeroScan leest de mesh van de Manifold (snel, bekabeld), leest het vluchtplan uit de KMZ op de RC (bestaande handmatige export), past alleen de gimbal-richting per waypoint aan, en stuurt de gewijzigde KMZ terug naar de Manifold via dezelfde bekabelde route. De radio zit niet in het bulk-datapad.**

---

## Wat we wilden bereiken

Een gebouwinspectie-missie die in AeroScan is gepland kunnen vliegen **zonder dat de piloot de controller hoeft te verlaten** — geen SD-kaart-omweg, geen "download KMZ → loop naar de controller → wissel kaart → herstart Pilot 2 → importeer" cyclus.

## Wat we daadwerkelijk hebben gebouwd en geleerd

**Werkt vandaag:**
- Draadloos pad RC ↔ Manifold over OcuSync (proof-of-concept) — **alleen behouden als kanaal voor controle-berichten**, niet voor bulkbestanden. Payloads ≤500 KB in <2 minuten; prima voor "vlieg missie X"-commando's en status-uitvragen.
- De Android-app op de controller (`rc-companion/`) registreert MSDK V5 schoon, koppelt aan de M4E, en stuurt data via DJI's MOP-kanaal naar een passieve Manifold-luisteraar (`rc_probe.c`). End-to-end geverifieerd.

**Gevalideerd door inspectie (2026-05-03):**
- De `/blackbox/`-map op de Manifold bewaart per-vlucht perceptie-data in standaard PLY-formaat. 18 GB verdeeld over 35+ vluchten. We hebben SSH-toegang. **Geen bestandsoverdracht nodig voor de mesh — we lezen die rechtstreeks.**
- DJI onderhoudt een `/blackbox/the_latest_flight`-symlink die naar de meest recente vlucht verwijst. **Geen vlucht-ID-opzoekmechanisme nodig.**
- Het versleutelde `expl_plan.bin.enc` is ondoorzichtig, dus het vluchtplan moet nog steeds uit de KMZ op de RC komen. Het bestaande USB-MTP-naar-laptop-proces van de piloot regelt dat — handmatig maar betrouwbaar.

**De kern-inzicht:** de juiste scope is **gimbal-aanvulling, niet missie-herplanning**. We nemen het Smart3D-vluchtpad zoals het is, extraheren facades uit de Manifold-mesh, en overschrijven alleen `gimbalPitchAngle` / `gimbalYawAngle` per waypoint zodat de camera op de dichtstbijzijnde zichtbare facade wordt gericht. De drone vliegt opnieuw een pad dat hij al weet uit te voeren.

## Productie-workflow (na pivot)

1. **In het veld:** piloot vliegt een Smart Auto-Exploration-missie zoals gewend. De mesh wordt opgebouwd in `/blackbox/the_latest_flight/`. De Smart3D-KMZ komt terecht in de RC-opslag.
2. **Bij de laptop (depot, hangar of elke locatie met een USB-kabel):**
   - USB-C-kabel van de laptop naar de M4E-debugport → Manifold verschijnt op `192.168.42.120` (door DJI gedocumenteerd voor pc's). Of, in het lab, het bestaande Wi-Fi-LAN.
   - Piloot verbindt RC met laptop via USB-MTP en sleept de Smart3D-KMZ naar de laptop (bestaande workflow).
   - Klik "Aanvullen met NEN-2767-gimbals" in AeroScan. Backend leest de mesh van de meest recente vlucht over de bekabelde verbinding, parst de KMZ voor waypoints, berekent welke facade elk waypoint moet bekijken, en schrijft een gewijzigde KMZ.
   - Klik "Naar drone sturen" — backend SCP't de gewijzigde KMZ naar `/open_app/dev/data/received/` op de Manifold.
3. **Voor opstijgen:** kabel loskoppelen. Drone aanzetten. Piloot tikt op de AeroScan PSDK-widget op Pilot 2's live-flight view → drone uploadt de aangevulde KMZ via `DjiWaypointV3_UploadKmzFile` → `DjiWaypointV3_Action(START)` → drone vliegt hetzelfde pad met camera's nu gericht op facades.

## Waarom dit beter is dan de alternatieven

- **Beter dan SD-kaart-sideload** omdat er niets fysiek van de RC af hoeft. De MTP-kabel van de piloot naar de laptop vervangt de SD-kaart-wissel, de laptop stuurt de aangevulde missie naar de drone via USB-C naar de aircraft-debugport, en Pilot 2 vliegt hem.
- **Beter dan draadloos transport** omdat OcuSync's ~5 KB/sec uplink 67 minuten zou doen over een 20 MB KMZ. Kabel-snelheid haalt meerdere MB/sec — bestanden in seconden.
- **Beter dan de planner naar de Manifold verhuizen** omdat we geen Python-op-Tegra-arm64-build hoeven te onderhouden. De laptop heeft de planner al.
- **Beter dan de missie vanaf nul herplannen** omdat de gimbal-aanvulpas veel eenvoudiger is dan NEN-2767 waypoint-generatie: zelfde pad, zelfde acties, alleen camera-hoeken veranderen.

## Wat we behouden uit de proof-of-concept

- De Android RC-companion-app (`rc-companion/`) blijft build-baar en op de plank voor toekomstige kleine-payload toepassingen (status-uitvragen, controle-commando's, ad-hoc tweaks in het veld). Niet op het kritieke productie-pad.
- De Manifold-zijde luisteraar (`rc_probe.c`) wordt in een follow-up PR uitgebreid om binnenkomende KMZ's daadwerkelijk naar disk weg te schrijven (vandaag logt die alleen hex-previews).
- Het bring-up-document (`rc-companion-bringup.md`) legt elke valkuil vast aan beide kanten, zodat dit transport later weer oppakken uren kost in plaats van dagen.

## Beslissing nodig

Bevestig dat we doorgaan met:
- **Bron-van-waarheid:** mesh van de Manifold (`/blackbox/the_latest_flight/dji_perception/1/`), vluchtplan uit de KMZ die de RC exporteert.
- **Transport:** USB-C-kabel (laptop ↔ M4E-debugport) voor SCP-push van de aangevulde KMZ. LAN als depot-fallback.
- **AeroScan-scope-verschuiving:** een "gimbal-aanvul"-pas bovenop Smart3D-missies, in plaats van NEN-2767-inspectiemissies vanaf nul genereren.

De technische proof-of-concept is klaar. Dit is nu een "waar besteden we de komende twee weken aan"-vraag — en het antwoord is de gimbal-aanvulpas + de laptop-zijde-ingester vanaf de Manifold, niet meer transport-plumbing.
