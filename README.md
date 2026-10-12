# Wind Turbine MPPT & Resistance Motor Controller

PID-basert reguleringssystem i Go for maksimal effektutvinning (MPPT) fra en småskala vindmølle. Systemet estimerer vindhastighet via dynamisk differensialtrykk (pitotrør/Bernoulli), beregner optimal rotorhastighet ($\lambda_{\text{opt}}$), og doserer generatorens motmoment ($\tau_e$) ved å modulere en variabel elektrisk lastmotstand ($R_{\text{last}}$). Prosjektet visualiseres med sanntids Chart.js-telemetri og en interaktiv Three.js 3D-visning.

<img width="944" height="490" alt="image" src="https://github.com/user-attachments/assets/077f7be7-4baf-46fb-b5dd-ba012b9bf8ee" />


---

## Matematiske modeller og formler

### 1. Trykkmåling til vindhastighet (Bernoullis ligning)
Nacellens pitotrør måler det dynamiske stagnasjonstrykket mot referansestatisk trykk:

$$\Delta P = \frac{1}{2} \rho v^2 \quad \implies \quad v = \sqrt{\frac{2 \Delta P}{\rho}}$$

* **$\Delta P$:** Målt differensialtrykk ($\text{Pa}$)
* **$\rho$:** Lufttetthet ($\approx 1.225\ \text{kg/m}^3$ ved havnivå og 15°C)
* **$v$:** Beregnet vindhastighet ($\text{m/s}$)

---

### 2. Aerodynamikk og optimal rotorhastighet (MPPT)
Rotorens spisshastighetsforhold (Tip-Speed Ratio, TSR) $\lambda$ defineres som forholdet mellom rotorbladenes periferihastighet og vindens hastighet:

$$\lambda = \frac{\omega \cdot R}{v}$$

* **$\omega$:** Rotorens vinkelhastighet ($\text{rad/s}$)
* **$R$:** Rotorbladets radius ($\text{m}$)

#### Turbinens virkningsgrad $C_p(\lambda)$
Tilgjengelig aerodynamisk effekt i luftstrømmen gjennom rotorarealet $A = \pi R^2$ følger Betz' lov:

$$P_{\text{aero}} = \frac{1}{2} \rho A v^3 C_p(\lambda) = \frac{1}{2} \rho \pi R^2 v^3 C_p(\lambda)$$

Virkningsgraden $C_p(\lambda)$ er teoretisk begrenset av Betz-grensen ($C_p \le \frac{16}{27} \approx 0.593$). I simulatoren modelleres profilen rundt optimal TSR ($\lambda_{\text{opt}} \approx 7.0$):

$$C_p(\lambda) = C_{p,\max} \cdot \exp\left(-\frac{1}{2}\left(\frac{\lambda - \lambda_{\text{opt}}}{\sigma}\right)^2\right)$$

#### Optimalt settpunkt for turtall
For å garantere drift i maksimalpunktet (Maximum Power Point Tracking) defineres kontrollerens målverdi:

$$\omega_{\text{target}} = \frac{\lambda_{\text{opt}} \cdot v}{R}$$

Aerodynamisk drivmoment som virker på akselen:

$$\tau_{\text{aero}} = \frac{P_{\text{aero}}}{\omega}$$

---

### 3. Elektromekanisk motstandsbrems (Generatormodell)
Generatoren er en permanentmagnetmaskin (DC/BLDC) koblet over en variabel passiv lastmotstand $R_{\text{last}}$ (f.eks. en MOSFET/PWM-chopper):

#### 1. Induseringsspenning (Back-EMF)
$$E = k_e \cdot \omega$$

* **$k_e$:** Motorens spenningskonstant ($\text{V}\cdot\text{s/rad}$)

#### 2. Kretsstrøm og indre tap
Med motorens indre viklingsmotstand $R_{\text{indre}}$ blir kretsens totale seriemotstand $R_{\text{tot}} = R_{\text{indre}} + R_{\text{last}}$:

$$I = \frac{E}{R_{\text{indre}} + R_{\text{last}}} = \frac{k_e \cdot \omega}{R_{\text{indre}} + R_{\text{last}}}$$

#### 3. Elektromagnetisk bremsemoment
$$\tau_e = k_t \cdot I = \frac{k_t \cdot k_e \cdot \omega}{R_{\text{indre}} + R_{\text{last}}}$$

* **$k_t$:** Motorens momentkonstant ($\text{N}\cdot\text{m/A}$, hvor $k_t = k_e$ i SI-enheter)

#### 4. Høstet elektrisk effekt
Effekten som leveres til lastmotstanden (forbruker/dump-load):

$$P_{\text{last}} = I^2 \cdot R_{\text{last}} = \left(\frac{k_e \cdot \omega}{R_{\text{indre}} + R_{\text{last}}}\right)^2 \cdot R_{\text{last}}$$

#### 5. Passiv reverseringssikring
Fordi kretsen utelukkende består av en passiv motstand, kan den aldri tilføre energi til generatoren:

$$\lim_{\omega \to 0} E = 0 \implies \lim_{\omega \to 0} I = 0 \implies \lim_{\omega \to 0} \tau_e = 0$$

Dersom rotoren stopper opp, faller motmomentet automatisk til null. Generatoren kan derfor aldri fungere som motor eller trekke rotoren i revers.

---

### 4. Rotordynamikk (Newtons 2. lov for rotasjon)
Rotorens vinkelakselerasjon bestemmes av differansen mellom aerodynamisk drivmoment, generatorens bremsemoment og mekanisk lagerfriksjon:

$$J \frac{d\omega}{dt} = \tau_{\text{aero}} - \tau_e - B \omega$$

* **$J$:** Rotorens og drivlinjens treghetsmoment ($\text{kg}\cdot\text{m}^2$)
* **$B$:** Mekanisk viskøs friksjonskoeffisient ($\text{N}\cdot\text{m}\cdot\text{s/rad}$)

I simulatoren diskretiseres ligningen med tidssteg $\Delta t$:

$$\omega_{k+1} = \max\left(0,\ \omega_k + \frac{\tau_{\text{aero}} - \tau_e - B \omega_k}{J} \cdot \Delta t\right)$$

---

### 5. PID-regulator for motstandsstyring (`pid.go`)

For å unngå singulariteter ved åpen krets ($R \to \infty$) regulerer PID-kontrolleren kretsens elektriske **admittans** ($G = \frac{1}{R_{\text{last}}}$).

#### Avvik (Error)
$$e(t) = \omega(t) - \omega_{\text{target}}$$

* $\omega > \omega_{\text{target}}$ (rotoren går for fort): $e(t) > 0 \implies$ PID øker admittansen $G$ (reduserer $R_{\text{last}}$), strømmen stiger og rotoren bremses.
* $\omega < \omega_{\text{target}}$ (rotoren går for sakte): $e(t) < 0 \implies$ PID reduserer admittansen $G$ (øker $R_{\text{last}}$), strømmen synker og rotoren kan akselerere.

#### Diskret PID-oppdatering
* **Proporsjonalledd:**
  $$P_k = K_p \cdot e_k$$
* **Integralledd:**
  $$I_k = I_{k-1} + e_k \cdot \Delta t$$
* **Derivatledd:**
  $$D_k = K_d \cdot \frac{e_k - e_{k-1}}{\Delta t}$$

Uavskåret admittansepådrag:

$$G_{\text{unclamped}} = P_k + K_i \cdot I_k + D_k$$

#### Metning og Anti-Windup Clamping
Admittansen avgrenses til motorens fysiske grenseverdier:

$$G_k = \max\left(G_{\min},\ \min\left(G_{\max},\ G_{\text{unclamped}}\right)\right)$$

hvor:
$$G_{\min} = \frac{1}{R_{\text{last},\max}}, \quad G_{\max} = \frac{1}{R_{\text{last},\min}}$$

Dersom pådraget er mettet ($G_{\text{unclamped}} \neq G_k$) og feilen har samme fortegn som pådraget, tilbakestilles integratoren (back-calculation):

$$I_k \leftarrow I_k - e_k \cdot \Delta t$$

Faktisk lastmotstand satt på aktuatoren:

$$R_{\text{last}} = \frac{1}{G_k}$$

---

## Egenskaper

- **MPPT Regulering (`internal/turbine/controller.go`):** Finner optimal rotorhastighet i sanntid basert på beregnet vindhastighet.
- **Passiv sikkerhet mot reversering (`internal/turbine/motor_physics.go`):** Elektromekanisk modell som garanterer at rotoren aldri drives baklengs.
- **PID med Anti-Windup (`internal/turbine/pid.go`):** Stabil integratorklemming under vindkast og metning.
- **Realistisk 3D-visning & Telemetri (`cmd/server/main.go`):** Three.js-scene med dagslyshimmel, konisk stålmast, aerodynamisk nacelle med nesekonus, roterende rotorblader og synkroniserte Chart.js-grafer.

---

## Forutsetninger

- **Go 1.22+** installert og lagt til i `PATH`.

---

## How to Run

### 1. Start simulatoren og webgrensesnittet
Kjør følgende kommando fra rotmappen (`wind-resistance-turbine`):

```bash
go run ./cmd/server
