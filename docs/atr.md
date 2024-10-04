# ATR

(work in progress documentation)

Originally developed by Dado Mista. In this package, ATR was modeled after real world vehicle dynamics, which should make ATR tunes more consistent for different riders and setups.

## Tuning the Easy Way

- Input your height [m], weight [kg] and board weight [kg] in Specs
- Find your motor in the table below and set Motor Torque Constant accordingly
<!-- - Check that Wheel Diameter (MotorCfg > Additional Info) is correct (~ 280-300 mm) -->


| Motor | Torque Constant |
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
| Hypercore N52 | 0.64 |

(table derived from values listed [here](https://pev.dev/t/common-motor-foc-ranges-resistance-inductance-flux-linkage/1771))

That's it, you should now have a usable ATR tune. Adjust strengths and speeds to your liking.

## Advanced Fine Tuning

**Prerequisites**
- Input your height [m], weight [kg] and board weight [kg] in Specs
- Double check your Wheel Diameter in MotorCfg > Additional Info
    - To measure this, mark a spot on your tire and a corresponding spot on a smooth ground, then ride forward for at least one full wheel revolution and mark where it lands
    - Calculate $D_\mathrm{eff} = \frac{l}{N \cdot \pi}$, where $l$ is the measured length and *N* the number of wheel revolutions
    - Another way is to compare VESC speed with GPS speed and adjust the diameter accordingly

**Motor Torque Constant**
- Locate Motor Flux Linkage in MotorCfg > FOC > General
- Ignore units, multiply it by 0.0225 and set that as your torque constant in the package
    - Example: If your flux linkage is 30 mWb, you would get 30 * 0.0225 = 0.675 Nm/A
- The proper calculation is $K_t = \frac{3}{2} \cdot \lambda \cdot P$, where $\lambda$ is the flux linkage [Wb] and $P$ the number of pole pairs.
- Cross reference the table above to sanity check
- In the future, this step could be skipped by accessing flux linkage directly

---

For the next few steps find a large, smooth and **flat** area. We'll need to tune 3 coefficients with the goal of minimizing magnitude of slope readings during flat ground riding. Luckily, it's straight forward to isolate their effects:

**Tire Rolling Resistance**
- Ride at a constant low speed (around 1 m/s or 1000 ERPM)
- Observe Slope in AppUI
- If it is *greater* than 0, *increase* Rolling Resistance and vice versa

**Aerodynamic Drag Coefficient**
- Ride at a constant high speed (5+ m/s or 5000+ ERPM, the more the better)
- Observe Slope in AppUI
- If it is *greater* than 0, *increase* Drag Coefficient and vice versa

**Acceleration Resistance**
- Accelerate and decelerate at various speeds
- Again observe Slope in AppUI
- If *positive* acceleration yields *positive* slope, *increase* Acceleration Resistance and vice versa

> TIP: Perform these tests in both directions, ideally multiple times to minimize the influence of external factors.

---

Now it should feel pretty good, but as a sanity check, there is one more thing you can do:

- Find some smooth hill, measure its true slope
- Ride over that spot and observe Slope in AppUI, it should match the true slope
- If it is *greater* than the true slope, you can proportionally *decrease* **all 4 coefficients**, that is:
    - Torque Constant, Rolling Resistance, Drag Coefficient and Acceleration Resistance
    - This should essentially scale all the coefficients to their "proper" values without affecting the ride feel (well, besides indirectly changing ATR strength etc.)
- However, by doing this you are now changing the torque constant, something that should be intrinsic to your motor

Feel free to do a bit of back and forth to average things out, real world is messy. Which brings me to...

**Tuning by Feel**
- If the board is nose-high at low speeds, increase Rolling Resistance (especially if you're riding mostly off-road, $C_\mathrm{roll}$ should be higher on dirt)
- If the board is nose-high only at high speeds, increase Drag Coefficient
- If the board gets nose-high/stiff when accelerating, increase Acceleration Resistance
> TIP: Temporarily increase ATR Strength to make its side effects more noticeable.

Here are some reference values for my setup:

| Setup  | $C_t$ (torque) | $C_r$ (rolling) | $C_d$ (drag) | $C_a$ (accel) |
| ----------- | ----------- | ----------- | ----------- | ----------- |
| CC v2 + Burris 7" @ 7 psi | 0.68 | 0.04 | 1.2 | 0.7 |

<!-- Here are some expected torque constant values for common motors. Derived from values listed in [this pev.dev post](https://pev.dev/t/common-motor-foc-ranges-resistance-inductance-flux-linkage/1771).

| Motor | Torque Constant |
| ----------- | ----------- |
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


## How Does It Work?

We can model longitudinal vehicle dynamics like this:

$$ F_\mathrm{total} = F_\mathrm{drive} - F_\mathrm{rolling} - F_\mathrm{drag} - F_\mathrm{gravity} $$

Where the individual terms are:

$$ F_\mathrm{total} = C_a \cdot m \cdot a $$

$$ F_\mathrm{drive} = \frac{I \cdot C_t}{r_\mathrm{eff}} $$

$$ F_\mathrm{roll} = C_\mathrm{roll} \cdot m \cdot g \cdot cos(\theta) $$

$$ F_\mathrm{drag} = \frac{1}{2} \cdot \rho \cdot C_d \cdot A \cdot v^2 $$

$$ F_\mathrm{gravity} = m \cdot g \cdot sin(\theta) $$




<!-- Which expands to:

$$ C_a \cdot m \cdot a = \frac{I \cdot C_t}{r_\mathrm{eff}} - C_\mathrm{roll} \cdot m \cdot g \cdot cos(\theta) - \frac{1}{2} \cdot \rho \cdot C_d \cdot A \cdot v^2 - m \cdot g \cdot sin(\theta) $$ -->





Where $m$ is the total mass, $C_a$ the acceleration resistance coefficient, $a$ the "real" acceleration from the wheel, $I$ the motor current, $C_t$ the motor torque constant, $r_\mathrm{eff}$ the effective tire radius, $C_\mathrm{roll}$ the rolling resistance coefficient, $g$ the gravity constant, $\theta$ the road slope, $\rho$ the air density, $C_d$ the drag coefficient, $A$ the frontal area, $v$ the velocity.

<!-- TODO mass only version -->
Rider's frontal area can be estimated from mass using a power law as $a \cdot  m ^ b$. If we assume human bodies scale proportionally, then $b = 2/3$. If we relate $m$ and $h$ using BMI calculation, we would get $b = 3/4$, which more or less corresponds with [some studies](https://link.springer.com/article/10.1007/s004210100424). $a$ is just a scaling factor that takes into account some average body shape and stance. Here's the proposed relationship:

$$ A = 0.02 \cdot m ^ {0.75} $$

To make frontal area properly dependent on rider's mass and height, we can estimate it as:

$$ A = k_A \cdot \sqrt{\frac{m \cdot h}{\rho_\mathrm{h}}} $$

Where $m$ is the rider's mass, $h$ the rider's height, $\rho_\mathrm{h}$ the the rider's density and $k_A$ a coefficient that corrects for an average rider's shape. If we model the rider as a square prism, cylinder or a spheroid, we would get $k_A$ ranging from 1 to 1.13.

After approximating $cos(\theta) \approx 1$ and solving for the slope, we get:

$$
\theta \approx sin^{-1} \left(
\frac{I \cdot C_t}{r_\mathrm{eff} \cdot m \cdot g} - C_\mathrm{roll} \cdot sign(v) - \frac{\rho \cdot C_d \cdot A \cdot v \cdot |v|}{2 \cdot m \cdot g} - \frac{C_a \cdot a}{g}
\right)
$$

$sin^{-1}(x)$ could be approximated by $x + a x^3$ or skipped completely. Coefficients can be combined and precomputed, so the actual runtime calculation should not be much worse than the original implementation and pretty much boils down to:

$$
\theta \approx
k_\mathrm{drive} \cdot T - k_\mathrm{roll} \cdot sign(v) - k_\mathrm{drag} \cdot v \cdot |v| - k_\mathrm{accel} \cdot a_g
$$

<!-- $$
\theta \approx 
\begin{bmatrix} k_\mathrm{drive} & - k_\mathrm{roll} & - k_\mathrm{drag} & - k_\mathrm{accel} \end{bmatrix}
\begin{bmatrix} T \\ sign(v) \\ v \cdot |v| \\ a_g \end{bmatrix}
$$ -->

Where $T$ is the torque, $v$ the velocity and $a_g$ the wheel's acceleration in [g].

## Notes

- I suspect there might be an additional resistance term linearly dependent on speed, but that would further complicate tuning. 
- Regarding acceleration resistance:
    - Even though $C_t$ acts similarly to $C_a$, it compromises the ratios between traction force and resistive forces. Using acceleration resistance instead should better isolate the influence of acceleration.
    - Things could be rewritten to have a master multiplier for the final slope instead of the torque constant, but for now I'm choosing to keep the coefficients somewhat grounded in reality.
    - I am not sure how to split the acceleration resistance into board and rider component. It should be mainly related to rotational inertia of the rotating parts, so probably a portion of the board's mass, but by leaning forward, the rider also has some rotational inertia and produces some extra aerodynamic drag. For simplicity, it currently uses the total mass.
    - For some reason, I need ~0.7, which is quite a bit less than I would expect, I have yet to figure out why.
- Rolling resistance also happens to represent slope offset in radians, so every 0.01 change in $C_r$ should offset the slope reading by roughly 0.6°.
