---
name: Vluchtreview 10 juli — Directiesamenvatting
description: Wat de testvluchten van 2026-07-10 ons leerden, wat er sindsdien gerepareerd is, en welke beslissing nodig is voor de volgende meetvlucht.
type: project
---

# AeroScan — Vluchtreview 10 juli · Directiesamenvatting

**Datum:** 2026-09-02
**Eén-zinsamenvatting:** *De drone vliegt onze aangevulde missie, maar op 10 juli keek de camera vaak niet waar we hem heen stuurden, sloeg een kwart van de foto's over en plande tegen de verkeerde scan — alle drie de oorzaken zijn nu gevonden, gerepareerd en offline bewezen; wat nog ontbreekt is één meetvlucht om het in de lucht te bevestigen.*

---

## Waar we staan

Op 10 juli vlogen we vijf keer rond een bestelbus en een boom op een parkeerplaats — een noodgreep, omdat er nog geen gebouw beschikbaar was. **Het doel blijft gebouwen.** Alles hieronder is daarop gericht; de bus was alleen het proefkonijn.

De vluchten zelf gingen goed: het toestel accepteert onze missie, vliegt hem autonoom, en pauzeren/hervatten werkt. Wat níet goed ging, zagen we pas toen we de data van het toestel zelf lazen in plaats van naar het live-beeld te kijken.

## Wat we leerden — in vijf regels

1. **Elke missie die dag plande tegen een verouderde scan.** Het toestel maakte 's ochtends twee scans; onze software keek alleen in map `1` en zag de tweede (in map `2`) niet. Gevolg op dezelfde vliegroute: **9 herkende gevelvlakken in plaats van 181.**
2. **104 van de 398 foto's zijn nooit gemaakt.** We vlogen 3 m/s met 1,7 m tussen waypoints — de camera kreeg 0,58 s per punt om te draaien én te fotograferen, en het toestel wachtte daar niet op.
3. **De camera stond gemiddeld 35° naast het doel.** De gimbal negeerde ons draaicommando en liep tegen zijn eigen aanslag (±60°). Op het live-beeld leek het goed: de groothoeklens hield het doel nét in beeld.
4. **Onze eigen controle zei "100% goed" terwijl het 35° mis was.** De maat was verkeerd gekozen.
5. **We hadden geen eigen registratie aan boord.** De enige waarheid stond in de foto's op de SD-kaart — die nog steeds niet veiliggesteld is.

## Twee dingen die we eerder verkeerd zeiden

Eerlijkheid over onze eigen conclusies is onderdeel van de methode:

- **"De scan was 15 uur oud."** Onjuist. De map-datum loog; de bestanden erin waren van diezelfde ochtend (2,2 uur oud). De veiligheidscheck op ouderdom werkte dus gewoon. Het echte probleem was de verkeerde map (punt 1).
- **"De 104 gemiste foto's zijn bewezen door de logs."** Te stellig. De logs van het toestel zijn op dit punt onbetrouwbaar. Het áántal klopt (foto's geteld); de oorzaak (te weinig tijd per waypoint) is de meest waarschijnlijke verklaring, geen bewijs.

---

## Wat er gerepareerd is

| Probleem | Oplossing | Status |
|---|---|---|
| Verkeerde scan-map | Boordsoftware zoekt in álle mappen en pakt de nieuwste scan | Gebouwd en getest op het toestel; **nog niet geïnstalleerd** |
| Foto's overgeslagen | Toestel **stopt nu bij elk waypoint** (officiële DJI-vliegmodus), richt, fotografeert, vliegt door | Gebouwd, getest, op het toestel gezet; **nog niet gevlogen** |
| Camera 35° mis | Geen apart gimbal-draaicommando meer; de neus van het toestel draagt de richting (was al gebouwd, nu ook nog nooit gevlogen) | **Nog niet gevlogen** |
| "100% goed"-maat | Vervangen door beeldresolutie (mm/pixel), aantal waarschuwingen, aantal richtingswisselingen en te-ver-picks op het pilootscherm | Gebouwd, getest |
| Geen controle vooraf | Drie nieuwe plan-controles: te weinig tijd per waypoint, draai die het toestel niet haalt, resolutie buiten spec — draaien nu ook in de boordsoftware | Gebouwd, getest |
| Geen registratie aan boord | Boordsoftware logt nu 10× per seconde gimbal-hoeken, stand en positie per waypoint naar een bestand — voortaan geen SD-kaart nodig om te weten waar de camera keek | Gebouwd, getest; **nog niet geïnstalleerd** |
| Camera kiest verkeerd doel | Richtlogica kiest nu voor de **hele route tegelijk** (kijkt vooruit en achteruit) en mag niet verder dan 14,6 m richten; daarbuiten houdt hij DJI's eigen camerastand | Gebouwd, getest — zie cijfers hieronder |

**Richtlogica, gemeten op de echte data van 10 juli (zelfde scan, zelfde 398 waypoints):**

| | vóór | nu |
|---|---|---|
| Picks op iets 20–30 m verderop | 39 | **0** |
| Eén-waypoint "uitschieters" | 1 | **0** |
| Doelwisselingen | 76 | 54 |
| Scherpe richtingsomslagen (>90°) | 23 | 14 |

De 14 resterende omslagen zijn overgangen tussen de bus en geparkeerde auto's midden op het terrein — een eigenschap van de proeflocatie, niet van de logica. Bij een gebouw bestaat die tussenzone niet.

---

## Hoe het nu werkt — per waypoint

```
   vlieg naar waypoint ──▶ STOP ──▶ neus wijst naar gevel
                                        │
                                        ▼
                                  gimbal kantelt (alleen als nodig)
                                        │
                                        ▼
                                  foto "wp123" ──▶ door naar volgende
                                        │
                     boordcomputer logt intussen: gimbal, stand, positie, waypoint
```

Vóór 10 juli vloog het toestel dóór elk waypoint zonder te stoppen. De route zelf (DJI's spiraal om het doel) verandert niet; alleen waar het stopt en kijkt.

## Wat is bewezen, wat waarschijnlijk, wat alleen een vlucht kan bewijzen

| | |
|---|---|
| **Bewezen** (documentatie + meetkunde) | DJI's stopmodus stopt het toestel bij het punt. 312 van de 397 trajecten gaven de camera <1 s. Verse scan geeft 181 vlakken en 1,97 mm/pixel (binnen de eis van 2,0). Alle controles draaien nu vóór de vlucht. |
| **Waarschijnlijk, niet bewezen** | Dat stoppen de gemiste foto's oplost. Scherpere foto's. Vliegtijd ≈ 20 min (was 5). |
| **Alleen in de lucht te bewijzen** | Dat de gimbal nu de neus volgt. Hoe het toestel zich houdt bij 398 keer stoppen en optrekken. |

## Wat het kost

- **Vliegtijd:** ≈ 20 minuten per missie in plaats van 5 — 62 % van de accugrens met boordcomputer. Past op één accu, met minder reserve.
- **Foto's:** 398 in plaats van 294, één rechte opname per waypoint.
- **Ontwikkeltijd besteed:** één dag review + reparatie (2 september), volledig offline op gearchiveerde data.

---

## Tijdlijn

```
2026-06-12  Eerste eigen missie gevlogen. Gimbal-motor overbelast (opgelost).
            ▼
2026-07-10  Vijf missies, bus + boom. Grond-als-gevel opgelost; gimbal-
            draaifout ontdekt via de foto's; scan-map-fout ontdekt.
            ▼
2026-09-02  Volledige review op de boordlogs. Drie oorzaken bevestigd,
            twee eigen claims teruggenomen, zeven reparaties gebouwd
            en offline gemeten. Software op het toestel gezet.
            ▼
  volgende  Meetvlucht: stopmodus + registratie aan boord.
```

## Volgende stappen

1. **Boordpakket installeren** (één handeling in DJI Pilot) — zonder dit blijven de scan-map-fix en de registratie inactief.
2. **Code vastleggen** in beide repositories (nu nog niet gecommit).
3. **Meetvlucht** in stopmodus, ~20 minuten. Slagingscriteria: 398 foto's, gimbal recht voor de neus, geregistreerde hoeken volgen de commando's.
4. **SD-kaart van 10 juli veiligstellen** — de enige registratie van de gimbalfout.
5. Zodra een gebouw beschikbaar is: dezelfde meetvlucht dáár. De bus heeft ons geleerd wat de software fout deed; het gebouw is het echte doel.

## Beslissing nodig

- **Akkoord voor de meetvlucht** in stopmodus, met de langere vliegtijd als bewuste keuze (kwaliteit boven snelheid).
- **Toegang tot een gebouw** voor de eerste echte test — de parkeerplaats heeft zijn nut gehad.
