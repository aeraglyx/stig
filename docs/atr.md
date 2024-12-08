# ATR "v2"

This package tries to model ATR after real world vehicle dynamics, which should make tunes more consistent for different riders and setups. To showcase its simplicity, the next section looks at how a beginner could tune this version of ATR.


## Tuning the Easy Way

- Input your weight [kg] and board weight [kg] in Specs
- Check that Wheel Diameter (Motor Cfg > Additional Info) is correct (280-300 mm)
- Find your motor in the following table and set Specs > Motor Torque Constant accordingly

<!-- | Motor | Torque Constant |
| ----- | --------------- |
| Phub188 | 0.52 |
| SuperFlux Mk1 | 0.54 |
| SuperFlux Mk2 (HS) | 0.57 |
| SuperFlux Mk2 (HT) | 0.66 |
| CannonCore V1 | 0.83 |
| CannonCore V2 | 0.68 |
| Hypercore | 0.61 |
| Hypercore N42 | 0.63 |
| Hypercore N48 | 0.64 |
| Hypercore N52 | 0.64 | -->

| Motor | Torque Constant |
| ----- | --------------- |
| Phub188 | 0.52 |
| SuperFlux Mk1 / Mk2 HS / Mk2 HT | 0.54 / 0.57 / 0.66 |
| CannonCore V1 / V2 | 0.83 / 0.68 |
| Hypercore Stock / N42 / N48 / N42 | 0.61 / 0.63 / 0.64 / 0.64 |

*Derived from values listed in this [pev.dev post](https://pev.dev/t/common-motor-foc-ranges-resistance-inductance-flux-linkage/1771), original data by Dado Mista.*

Assuming you have a typical setup, that's it - you should now have a usable ATR tune. Adjust strengths and speeds to your liking, strength of 1 means parallel to the ground (typical values 0.0 - 0.5).


## Advanced Fine Tuning

**Prerequisites**
- Input rider mass [kg] and board mass [kg] in Specs
- Double check your Wheel Diameter in Motor Cfg > Additional Info
    - To measure this, mark a spot on your tire and a corresponding spot on a smooth ground, then ride forward for a few wheel revolutions and mark where it lands
    - Calculate $D = \frac{l}{N \cdot \pi}$, where $l$ is the measured length and *N* the number of wheel revolutions
    - Another way is to compare VESC speed with GPS speed and adjust the diameter accordingly

**Motor Torque Constant**
- Locate Motor Flux Linkage in Motor Cfg > FOC > General
- With units in mind, multiply it by 22.5 and set that as your torque constant in the package
    - Example: If your flux linkage is 30 mWb, you would get 0.001 * 30 * 22.5 = 0.675 Nm/A
    - The proper calculation is $C_t = \frac{3}{2} \cdot \lambda \cdot P$, where $\lambda$ is the flux linkage [Wb] and $P$ the number of pole pairs.
- Cross reference the table above as a sanity check
- In the future, this step could hopefully be skipped by accessing the flux linkage directly

---

For the next few steps find a large, smooth and **flat** area. We'll need to tune 3 coefficients with the goal of minimizing magnitude of the slope reading during flat ground riding. Luckily, it's straight forward to isolate the individual components:

**Tire Rolling Resistance**
- Ride at a constant low speed (around 1 m/s or 1000 ERPM)
- Observe Slope in AppUI
- If it is *greater* than 0, *increase* Rolling Resistance and vice versa

**Aerodynamic Drag Coefficient**
- Ride at a constant high speed (5+ m/s or 5000+ ERPM, the faster the better)
- Observe Slope in AppUI
- If it is *greater* than 0, *increase* Drag Coefficient and vice versa

**Acceleration Resistance**
- Accelerate and decelerate at various speeds
- Again observe Slope in AppUI
- If *positive* acceleration yields *positive* slope, *increase* Acceleration Resistance and vice versa
- This is an important coefficient, see *Tuning by Feel* below for a way to double check this

> TIP - Perform these tests in both directions to minimize the influence of external factors.

---

**Tuning by Feel**
- If the board is nose-high at low speeds, increase Rolling Resistance
- If the board is nose-high only at high speeds, increase Drag Coefficient
- If the board is nose-high/stiff when accelerating, increase Acceleration Resistance
> TIP - Temporarily increase ATR Strength to make its side effects more noticeable. On flat ground, ATR on should feel similar to ATR off.

Here are some reference values for my setup:

| Setup  | $C_t$ (torque) | $C_r$ (rolling) | $C_d$ (drag) | $C_a$ (accel) |
| ----------- | ----------- | ----------- | ----------- | ----------- |
| CC v2 + B7 @ ~5 psi | 0.68 | 0.05 | 1.25 | 0.7 |


## How Does It Work?

We can model longitudinal vehicle dynamics like this:

$$ F_\mathrm{total} = F_\mathrm{drive} - F_\mathrm{rolling} - F_\mathrm{drag} - F_\mathrm{gravity} $$

Where the individual terms are:

$$ F_\mathrm{total} = C_a \cdot m \cdot a $$

$$ F_\mathrm{drive} = \frac{I \cdot C_t}{r} $$

$$ F_\mathrm{roll} = C_\mathrm{roll} \cdot m \cdot g \cdot cos(\theta) $$

$$ F_\mathrm{drag} = \frac{1}{2} \cdot \rho \cdot C_d \cdot A \cdot v^2 $$

$$ F_\mathrm{gravity} = m \cdot g \cdot sin(\theta) $$

Where $m$ is the total mass, $C_a$ the acceleration resistance coefficient, $a$ the "real" acceleration, $I$ the motor current, $C_t$ the motor torque constant, $r$ the effective tire radius, $C_\mathrm{roll}$ the rolling resistance coefficient, $g$ the gravity constant, $\theta$ the road slope, $\rho$ the air density, $C_d$ the drag coefficient, $A$ the frontal area, $v$ the velocity.

Rider's frontal area can be estimated from mass using a power law $a \cdot  m ^ b$. This is further explored in [this GitHub repository](https://github.com/aeraglyx/frontal_area), where I tried to find the coefficients from real world measurements. I'm sure it could be improved, but for now, here is the proposed relationship:

$$ A = 0.02 \cdot m ^ {\frac{3}{4}} $$

We could incorporate rider height into the calculation, but it's probably not worth it considering it would be used solely for drag calculation (drag coefficient can include any frontal area inconsistencies).

After approximating $cos(\theta) \approx 1$, taking into account the sign of speed and solving for the slope, we get:

$$
\theta \approx sin^{-1} \left(
\frac{I \cdot C_t}{r \cdot m \cdot g} - C_\mathrm{roll} \cdot sign(v) - \frac{\rho \cdot C_d \cdot A \cdot v \cdot |v|}{2 \cdot m \cdot g} - \frac{C_a \cdot a}{g}
\right)
$$

$sin^{-1}(x)$ could be approximated as $x + a x^3$ or skipped completely. Coefficients can be combined and precomputed, so the actual runtime calculation should not be much worse than the original implementation and pretty much boils down to:

$$
\theta \approx
k_\mathrm{drive} \cdot T - k_\mathrm{roll} \cdot sign(v) - k_\mathrm{drag} \cdot v \cdot |v| - k_\mathrm{accel} \cdot a_g
$$

<!-- $$
\theta \approx 
\begin{bmatrix} k_\mathrm{drive} & - k_\mathrm{roll} & - k_\mathrm{drag} & - k_\mathrm{accel} \end{bmatrix}
\begin{bmatrix} T \\ sign(v) \\ v \cdot |v| \\ a_g \end{bmatrix}
$$ -->

Where $T$ is the torque (current times torque constant), $v$ the velocity and $a_g$ the acceleration in [g].


## Notes

- What's the point of this? I wanted to achieve 2 things:
    - Simpler, more accurate and predictable ATR
    - ATR depends on riders weight, so if we properly relate all the parameters to it and find good defaults for the rest, a newcomer could really easily achieve a good ATR tune by changing only one known value.
- Regarding acceleration resistance:
    - For some reason, I needed ~0.7 for both my ADVs, which is somewhat unexpected. I'd love to get a larger sample size. The idealized value is 1.0, but since there is rotational inertia in the system, it should be slightly higher than that. Might be related to the [ERPM correction](https://pev.dev/t/subtracting-boards-angular-velocity-from-erpm-to-improve-atr/1737), or possibly a bug.
    - I don't know how to properly split the acceleration resistance into board and rider components. It should be mainly related to rotational inertia of the rotating parts, so probably a portion of the board's mass, but by leaning forward, the rider also has some rotational inertia (and produces extra aerodynamic drag). For simplicity, it currently uses the total mass.
- Fun fact - Rolling resistance happens to represent slope offset in radians, so every 0.01 change in $C_r$ should offset the estimated slope by roughly 0.6°.
- TODO - Increase wheel radius based on speed as the tire expands?


## Acknowledgements

ATR was originally developed by Dado Mista.
