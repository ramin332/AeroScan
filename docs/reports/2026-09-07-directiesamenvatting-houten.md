---
name: Vluchtreview 7 september Houten — Directiesamenvatting
description: Drie vluchten boven hetzelfde huis in Houten. Wat er werkt, wat de foto's bewijzen, wat het kost aan vliegtijd, en één fout die we in onze eigen data vonden.
type: project
---

# AeroScan — Vluchtreview 7 september, Houten · Directiesamenvatting

**Datum:** 2026-09-07
**Eén-zinsamenvatting:** *Voor het eerst boven echte woonhuizen: de drone vloog drie missies achter elkaar, maakte élke geplande foto, de camera keek exact waar wij hem heen stuurden — en één instelling bracht het aantal gefotografeerde gevelvlakken van 5 naar 71, tegen de prijs van drie keer zoveel vliegtijd.*

---

## Waar we staan

Op 10 juli vlogen we nog rond een bestelbus op een parkeerplaats. Op 7 september vlogen we boven **echte huizen in Houten** — drie locaties, zeven vluchten, en aan het eind drie missies achter elkaar boven hetzelfde huis om ze eerlijk te kunnen vergelijken.

Alles wat op 2 en 3 september gebouwd was, vloog die dag voor het eerst. Het werkte.

## De vergelijking die we opsturen

Drie vluchten, hetzelfde huis, dezelfde scan, dezelfde 276 vliegpunten. Alleen de instellingen verschillen.

| | Vlucht | Route | Snelheid | Stopt bij elk punt? | Foto's per punt | Foto's |
|---|---|---|---|---|---|---|
| **A** | DJI Smart 3D, standaard | van DJI | van DJI | nee | rozet van 5 | 635 |
| **B** | AeroScan | zelfde 276 punten | 1,0 m/s | nee | 1 | 276 |
| **C** | AeroScan | zelfde 276 punten | 2,0 m/s | **ja** | **2** | 539 |

**A** is de standaard die DJI zelf levert. **B** en **C** zijn van ons. Beide vliegen **DJI's eigen route** — AeroScan verplaatst de drone niet, het bepaalt alleen waar de camera kijkt en wanneer hij afdrukt.

## Het resultaat in vijf regels

1. **Elke geplande foto is gemaakt.** 276 van 276, en 539 van 539. Op 10 juli ontbraken er nog 104 van de 398.
2. **De camerafout van 10 juli is weg.** De gimbal stond toen gemiddeld 51,5° scheef en liep tegen zijn aanslag. Nu: **6,5°**, en géén enkele foto tegen de aanslag.
3. **De camera doet exact wat we vragen.** Verschil tussen commando en werkelijkheid: **0,1°** in kantelhoek, en **0,1°** in draaihoek bij de tweede foto.
4. **Stoppen en een tweede foto maken is dé sleutel tot dekking.** Van **5 naar 71** gefotografeerde gevelvlakken (≥ 2 m²) op precies dezelfde route.
5. **Beeldresolutie binnen de eis.** 1,85 mm/pixel tegen een norm van 2,0 — met de gewone groothoeklens, omdat DJI's route dicht langs deze huizen vliegt.

## Wat het kost: eerlijk gemeten

Wij schreven eerst dat vlucht C "twee keer zo snel" was, omdat hij zijn trajecten met 2 m/s vliegt in plaats van 1. **Dat was fout, en de piloot corrigeerde het.** C stopt bij alle 276 punten, en dan telt de vliegsnelheid nauwelijks. Gemeten van de eerste tot de laatste foto:

| | foto's | tijd in de lucht | per vliegpunt |
|---|---|---|---|
| A — DJI, stopt nooit | 635 | **6,5 min** | — |
| B — doorvliegen | 276 | **6,7 min** | 1,4 s |
| C — stoppen + 2 foto's | 539 | **21,2 min** | 4,6 s |

C duurt dus **3,2 keer zo lang**. Van die 4,6 seconden per punt gaat er 0,7 s naar vliegen, **1,0 s naar de twee foto's** en **2,3 s naar nauwkeurig aankomen en stilhangen**. Het fotograferen is goedkoop; het stoppen is duur.

De afweging blijft gunstig — 14× de dekking voor 3,2× de tijd is **4,4× meer dekking per minuut** — maar het is een afweging, geen gratis winst. Bij een groter gebouw past dit niet meer op één accu.

## Een fout die we in onze eigen data vonden

Na afloop lazen we de werkelijke camerahoeken uit de foto's. Daar bleek iets dat op geen enkel scherm te zien was:

**De eerste foto van elk punt werd genomen terwijl de gimbal nog scheef stond van de vorige.** De tweede foto draait de gimbal opzij naar een tweede muur — maar niets draaide hem daarna terug. Het verband is onmiskenbaar: samenhang **+0,985** tussen de zijstand bij het vorige punt en de scheefstand bij het volgende. Waar de vorige draai groot was, stond de camera **36,6°** naast het doel.

Gevolg: de eerste foto's misten hun doel met gemiddeld 13,6°, terwijl dezelfde missie zonder tweede foto op 6,7° zat. Op het live-beeld valt dat niet op — de groothoeklens houdt de muur nét in beeld. **Dezelfde valkuil als in juli: "in beeld" is niet "goed gericht."**

Dit is inmiddels gerepareerd: de gimbal wordt nu expliciet op de eerste muur gericht vóór de eerste foto, met een begrenzing zodat hij nooit tegen zijn aanslag kan lopen. Alléén de gimbal beweegt; de drone zelf draait niet. **Nog niet gevlogen.**

## Wat we eerder verkeerd zeiden — en nu terugnemen

- **"Het toestel negeert ons gimbal-draaicommando."** Dat concludeerden we op 10 juli. **Onjuist.** Vlucht C stuurde 256 keer een absolute draaihoek en de gimbal kwam er tot op **0,1°** uit. De oorzaak van juli lag elders. Dat opent een weg die we hadden afgesloten.
- **"C is twee keer zo snel."** Onjuist, zie hierboven. C is 3,2× langzamer.

Beide correcties staan ook in het technische rapport in het pakket.

---

## Wat er nog niet goed is

| | |
|---|---|
| **Dekking blijft onder de helft** | 71 van 162 gevelvlakken. De grens zit in de richtlogica (één hoofddoel per punt), niet in de herkenning. |
| **Registratie aan boord ontbrak** | De boordcomputer schreef geen enkel meetbestand tijdens de drie vluchten. Onopgelost. |
| **Boordlogboek verdwenen** | Het logboek van het toestel wordt bij uitschakelen gewist; onderweg naar huis ging de hele dag verloren. Voortaan uitlezen vóór het afsluiten. |
| **Eén punt van DJI's route ligt 1,8 m van de muur** | Onze planning verplaatst geen punten; dit is DJI's eigen route. Goed om te weten. |

## Volgende stappen

1. **Meetvlucht met de gerepareerde richting** — zelfde huis, zelfde instellingen als C. Slagingscriterium: eerste foto's binnen ~5° van hun doel in plaats van 13,6°.
2. **Onderzoeken of stoppen echt nodig is.** De twee foto's kosten samen 1,0 s; een traject van 1,5 m bij 1 m/s duurt 1,5 s. Mogelijk kan het zonder stoppen, en dan gaat 21 minuten terug naar ongeveer 7. DJI's eigen missie fotografeert 635 keer zonder ooit te stoppen — het kan dus, alleen niet met hun methode.
3. **Registratie aan boord repareren**, zodat we niet van de SD-kaart afhankelijk zijn.
4. **Dekking verhogen** via de richtlogica.

## Beslissing nodig

- **Akkoord voor één meetvlucht** om de richtingsreparatie te bevestigen (~20 min, zelfde locatie).
- **Keuze in de afweging tijd tegen dekking:** accepteren we 21 minuten voor 71 vlakken, of investeren we eerst in het wegnemen van de stoptijd? Onze aanbeveling: eerst de richting bevestigen, daarna pas aan snelheid werken — anders maken we een missie sneller die nog niet goed kijkt.
