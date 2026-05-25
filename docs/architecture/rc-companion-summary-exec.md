---
name: RC-Companion Directiesamenvatting v2
description: Wat we hebben gebouwd, waarom, en waar we staan na de eindspurt 2026-05-03 → 2026-05-07.
type: project
---

# RC-Companion — Directiesamenvatting v2

**Datum:** 2026-05-07 (vervangt v1 van 2026-05-03)
**Eén-zinsamenvatting:** *Een drone scant een gebouw, de RC stuurt het scan-resultaat naar de boord-computer, die maakt er een NEN-2767-quality inspectiemissie van, en de piloot keurt die goed op de RC zelf — geen laptop, geen kabel, geen SD-kaart.*

---

## Het probleem

DJI's Smart Auto-Exploration zet de drone razendsnel om een gebouw heen voor een fotogrammetrie-scan. Het resultaat is mooi, maar **niet bruikbaar voor NEN-2767-inspectie**: de camera staat schuin, de waypoint-dichtheid is verkeerd, en defecten op gevels zijn niet leesbaar.

Onze taak: van die scan een bruikbare inspectievlucht maken — **dezelfde route, andere camera-instellingen** — zonder dat de piloot terug naar kantoor hoeft.

## Hoe het werkt — overzicht

```
   ┌─────────────┐     scan      ┌──────────────┐
   │  M4E drone  │  ──────────▶  │  /blackbox/  │  (mesh blijft op de boord-computer)
   └─────────────┘               └──────┬───────┘
                                        │
                                        ▼
   ┌─────────────┐  1. AUGM      ┌──────────────┐
   │  RC + onze  │  ─────────▶   │   Manifold   │  ← 2. mesh + scan combineren,
   │  Android-   │               │ (boord-      │     gevels detecteren,
   │  app        │  ◀─────────   │  computer)   │     camera per waypoint herrichten
   └─────────────┘  3. PRVW      └──────────────┘
        │
        │  4. piloot tikt "Goedkeuren"
        ▼
   ┌─────────────┐  5. EXEC      ┌──────────────┐
   │  RC         │  ─────────▶   │  M4E drone   │  ← 6. KMZ in geheugen
   └─────────────┘               └──────────────┘
        │
        ▼  (Pilot 2)
   tap "AeroScan: Fly" widget → drone vliegt de aangevulde missie
```

**Wat is er nieuw t.o.v. v1?** v1 zei: "de bandbreedte over de radio is te krap, we doen het via een kabel naar een laptop in het veld." Met twee meetresultaten kantelde dat: een *fingerprint* van de scan-wolk past in 121 KB, de hele missie-intent in 71 KB. Samen ~40 seconden uplink. **Geen laptop meer nodig.**

---

## Wat we hebben gebouwd

### 1. Augmenter op de boord-computer
De Python-engine die uit de webapp komt, draait nu ook op de Manifold. Hij leest de scan-mesh die DJI op `/blackbox/` achterlaat, lijnt die uit met de fingerprint van de RC, detecteert gevels, en rekent voor elke waypoint uit waar de camera precies heen moet wijzen.

**Tijd per missie:** ~2-4 minuten voor een typisch gebouw (Mijande: 1233 waypoints, 1907 gevel-vlakken).

### 2. RC-companion app (Android, op de RC zelf)
- Leest de Smart3D-KMZ van het RC-bestandssysteem
- Stuurt scan + missie naar de Manifold over de DJI-radio
- Toont een **preview-kaart** met statistieken vóór goedkeuring (hoeveel waypoints, hoeveel gevels gevonden, hoeveel "verdachte" camera-poses)
- Twee grote knoppen: **GOEDKEUREN** of **AFWIJZEN**

### 3. Boord-computer software (PSDK, in C)
Wacht op de RC, draait de augmenter, stuurt het resultaat terug, en — als de piloot goedkeurt — laadt de aangevulde missie direct in het toestel via de officiële DJI-API. Daarna verschijnt er een knop "AeroScan: Fly" op het Pilot 2-scherm.

### 4. Verificatie-script
`scripts/verify_augmented_kmz.py` opent een aangevulde missie en checkt **per waypoint of de camera daadwerkelijk naar de gevel wijst** — niet zomaar in een richting. Belangrijk omdat aggregaten ("gemiddelde camera-hoek") problemen verbergen die individuele uitschieters hebben.

---

## Wat is er getest? Wat niet?

| | Status |
|---|---|
| Scan op aircraft (Smart Auto-Exploration) | Werkt — al jaren productie bij DJI |
| RC stuurt scan + missie naar Manifold (~40 sec) | **Getest, werkt** |
| Manifold rekent de aanvulpas uit (~3 min) | **Getest, werkt** |
| Preview-kaart op de RC met statistieken | **Getest, werkt** |
| Goedkeuren/afwijzen vanaf de RC | **Getest, werkt** |
| Aangevulde missie in het toestel laden | **Getest, MD5 klopt** |
| Camera wijst naar de juiste gevel | **97 % goed** (controle-script), 3 % nog mis-aim |
| Widgets op Pilot 2 (live view) | **Opgelost (2026-05-25):** widgets verschijnen én zijn interactief **mits de app als DPK draait** (niet als ruwe `systemd`-binary). De stock gimbal-widget werkte. Een eigen "AeroScan: Fly"-widget is nog te bouwen. |
| **Daadwerkelijk vliegen met aangevulde missie** | **Nog niet uitgevoerd.** Propellers zijn er bewust afgehouden tijdens de pipeline-test. |

---

## Waarom de aanpak zoals hij is

| Keuze | Waarom |
|---|---|
| **Geen laptop in het veld** | Eén apparaat minder om kwijt te raken, op te laden of te koppelen. De radio is er al, de RC heeft al een scherm. |
| **Mesh op de Manifold laten staan, niet over de radio sturen** | De volledige scan-mesh is ~1 GB. Versturen zou minutenlang duren of gewoon falen. We sturen alleen een fingerprint van 121 KB om de twee wolken op elkaar uit te lijnen. |
| **Piloot goedkeurt voordat de drone iets doet** | Als de aanvulpas een vreemde camera-pose oplevert, ziet de piloot dat in de preview en kan hij afwijzen. Veiligheid voor afwijking. |
| **DJI Pilot 2 blijft de vlieg-app** | We voegen één widget toe; we vervangen Pilot 2 niet. Piloot houdt zijn vertrouwde tooling. |
| **Aangevulde missie in plaats van vanaf nul** | DJI's vluchtroute is veilig geprobeerd op duizenden gebouwen. We veranderen alleen *waar de camera kijkt*, niet waar de drone heen vliegt. Dat is een veel kleiner risico-oppervlak. |
| **Iteratief de cameraregels verfijnen** | Eerste poging: camera dichtstbijzijnde gevel. Eindversie: 3D-afstand-tot-gevel + lichte voorkeur voor verticale wanden + gladstrijken over 5 waypoints + uitschieter-correctie. Elke verfijning kwam door visueel te kijken naar het resultaat in de viewer en bugs te repareren die niet uit de aggregaat-statistieken te zien waren. |

---

## Wat we hebben behouden uit v1

- De optie om de aangevulde missie via een **kabel** terug te zetten blijft technisch werkend als depot-tooling, voor het geval de radio ergens niet werkt.
- De webapp blijft het primaire platform voor missies waar **geen Smart3D-vlucht aan voorafgaat**. De gimbal-aanvulpas is specifiek voor de "ik heb net een Smart3D-scan gevlogen, geef me een NEN-2767-herhaling"-workflow.

## Wat is verschoven

- De **Android-app op de RC** is geen proof-of-concept meer. Hij staat in het kritieke pad.
- De **OcuSync-radio** draagt nu de complete payload (~510 KB heen + terug). Met de fingerprint-strategie past dat ruim binnen het bandbreedte-budget.

---

## Tijdlijn van de eindspurt

```
2026-05-03  v1 vastgelegd: "kabel + laptop, radio te krap"
            ▼
2026-05-04  Bench-meting: fingerprint past in 121 KB. Architectuur kantelt.
            ▼
2026-05-05  Augmenter installeerbaar op aarch64. Eerste end-to-end op
            Mijande-data via SSH (geen RC nog).
            ▼
2026-05-06  RC-app klaar (parser + AUGM/PRVW/EXEC + preview-scherm).
            's Avonds: eerste echte radio-test op de M4E. Pipeline werkt
            tot READY_TO_FLY (props off).
            ▼
2026-05-07  Bug-jacht op de camera-poses: 4 iteraties van smoothing,
            cloud-prep matchen op de webapp, verificatie-script.
            Eindresultaat: 97 % camera's correct gericht op gevels.
```

---

## Volgende stappen

1. **Werkelijk vliegen op Mijande** — alle data ziet er goed uit, één props-on flight bevestigt het.
2. ~~**Pilot 2 widget UI verkennen**~~ **Opgelost (2026-05-25):** de boord-app is nu als **DPK** geïnstalleerd (`dji_app_ctl install -i <file>.dpk`, beheerd via Pilot 2) en widgets renderen interactief op de live view. De ruwe `systemd`-binary deed dat niet. Deployment-model: `manifold-deployment.md`. Resteert: een eigen "Fly"-widget bouwen.
3. **Tweede en derde locatie** — Mijande is één gebouw. Elk gebouw heeft eigen detectie-uitdagingen (overhangs, balkons, dakdetails).
4. **Anomalie-drempels kalibreren** — na 5-10 echte vluchten weten we wat de piloot daadwerkelijk als waarschuwing wil zien.
5. **Manifold disk-management** — `/blackbox/` groeit ~1 GB per vlucht. Retentie-script nodig vóór de 50ste vlucht.

---

## Beslissing nodig

**Geen.** v1 vroeg om bevestiging om kabel + laptop te gebruiken. v2 heeft op echte hardware bewezen dat dat niet hoeft. De volgende stap is een echte vlucht — geen architectuurwijziging.
