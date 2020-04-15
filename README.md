# PMPS-Homework-2
Fakultet elektrotehnike Tuzla - Projektovanje mikroprocesorskih sistema


Za tehnološki reaktor dat na slici potrebno je razviti upravljački sistem baziran na STM
mikrokontroleru. Upravljački sistem se sastoji od STM32 kontrolera koji realizuje real-time
upravljačke funkcije i računara koji realizuje HMI (Human-Machine-Interface) funkcije operatorskog
nadzora i vizuelizacije parametara procesa. Vaš zadatak je razvoj aplikaciju na STM32 kontroleru.

STM32 real-time aplikaciju treba struktuirati kao multi-tasking aplikaciju korištenjem freeRTOS
operativnog sistema (kernela) sa odgovarajudim CMSIS RTOS API wrapperima. Aplikacija treba da
ima četiri taska:

**Task1** izvodi upravljanje (regulaciju) nivoa fluida u reaktoru tako što mjeri nivo u reaktoru (analogni
signal sa senzora), te sa PID regulatorom manipuliše analognim aktuatorom (proporcionalni servo
ventil) koji reguliše dotok baznog fluida u reaktor (rezervoar). Ovaj task je periodični task sa
periodom T1 = 100ms, i odgovarajudim prioritetom.


**Task2** izvodi upravljanje (regulaciju) pH vrijednosti fluida u reaktoru tako što mjeri pH vrijednost u
reaktoru (analogni signal sa senzora), te sa PID regulatorom manipuliše analognim aktuatorom
(proporcionalni servo ventil) koji reguliše dotok fluida za neutralizaciju u reaktor (rezervoar). Ovaj
task je periodični task sa periodom T1 = 50ms, i odgovarajudim prioritetom.

**Task3** izvodi upravljanje (regulaciju) temperature fluida u reaktoru tako što mjeri temperaturu
fluida u reaktoru (analogni signal sa senzora), te sa PID regulatorom manipuliše PWM aktuatorom
(solid-state switched grijač) Ovaj task je periodični task sa periodom T1 = 70ms, i odgovarajudim
prioritetom.

**Task4** izvodi komunikacione funkcije, tj. funkcije razmjene poruka sa nadzornim HMI računarom,
korištenjem UART serijskog interfejsa. HMI računar može slati poruke STM32 računaru za
postavljenje referentnih vrijednosti procesnih varijabli (nivo, pH, temperatura). Ove poruke imaju
format:

Prvi bajt, kod
procesne varijable
(npr. 1,2,3)

```
Referentna
vrijednost
procesne varijable
u tekst formatu
%3.3 f
```
STM32 šalje poruke na HMI o vrijednostima procesnih varijabli (vizualizacija, data logging, etc.).
Ove poruke imaju format:

Prvi bajt, kod
procesne varijable
(npr. 1,2,3)

```
Referentna vrijednost
procesne varijable u tekst
formatu %3.3f
```
```
Stvarna
(mjerena)
vrijednost
procesne
varijable u tekst
formatu %3.3 f
```
```
Time stamp
varijabli u
tekst formatu
%10d
```
Ovaj task je event-driven task odgovarajudeg prioriteta.

## NAPOMENA:

Digitalni PID regulator se realizuje kao:

Računanje regulacione greške e(k) = r(k) – y(k)

Gdje je r(k) referentna vrijednost regulatora u trenutku kT, a y(k) je stvarna vrijednost varijable u
trenutku kT.


Te računanje upravljačkog signala kao:

u(k) = u(k-1) + b0*e(k)+b1*e(k-1)+b2*e(k-2)

u svakom trenutku uzorkovanja (kT). Gdje su u(k-1), e(k-1), e(k-2) prethodne vrijednosti upravljanja
i greške, a b0, b1, b2 su projektovani parametri regulatora.


