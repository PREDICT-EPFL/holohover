# Puck collision models and bank-shot planning

Summary of the old and new puck impact models, of how the mirror trick of the shot planner
carries over to the new wall model, and of the constraints that keep the planner's
predictions exact.

## 1. Notation

All impacts are 2D, instantaneous and act at a single contact point. Puck quantities have no
subscript, hovercraft quantities carry the subscript $H$.

| Symbol | Meaning |
|---|---|
| $m$, $r$, $I = \kappa m r^2$ | puck mass, radius, inertia ($\kappa = 1/2$ for a uniform disk) |
| $v$, $\omega$ | puck velocity and spin (counter-clockwise positive) |
| $m_H$, $r_H$, $I_H = \kappa_H m_H r_H^2$, $v_H$, $\omega_H$ | the same for the hovercraft |
| $n$ | unit contact normal, pointing towards the puck (into the table for a wall, from the hovercraft to the puck for a hit) |
| $t = \hat z \times n$ | unit tangent |
| $u_n$, $u_t$ | velocity of the puck's contact point relative to the other body's contact point, along $n$ and $t$ |
| $P_n$, $P_t$ | normal and tangential impulse on the puck; the other body gets the opposite |
| $M_n$, $M_t$ | effective masses at the contact, normal and tangential |
| $x'$, $\Delta x = x' - x$ | value after the impact, change |

$u_n < 0$ while the bodies approach, and $u_t$ is the slip: how fast the puck's rim slides
along the other surface.

| | hit | wall |
|---|---|---|
| $u_n$ | $(v - v_H)\cdot n$ | $v \cdot n$ |
| $u_t$ | $(v - v_H)\cdot t - r\omega - r_H\omega_H$ | $v \cdot t - r\omega$ |
| $1/M_n$ | $1/m + 1/m_H$ | $1/m$ |
| $1/M_t$ | $(1 + 1/\kappa)/m + (1 + 1/\kappa_H)/m_H$ | $(1 + 1/\kappa)/m$, so $M_t = m/3$ |

Whatever the impulses, the bodies respond as rigid bodies:

```math
\Delta v = \frac{P_n n + P_t t}{m}, \qquad \Delta\omega = -\frac{r P_t}{I}, \qquad
\Delta v_H = -\frac{P_n n + P_t t}{m_H}, \qquad \Delta\omega_H = -\frac{r_H P_t}{I_H},
```

which changes the relative velocity at the contact by

```math
\Delta u_n = \frac{P_n}{M_n}, \qquad \Delta u_t = \frac{P_t}{M_t}.
```

**Effective masses.** Mass is isotropic for the centre of mass, but the impulse acts on the
rim. A normal push goes through the centre and only translates the disk. A tangential push
also spins it, so the rim gives way faster: per unit impulse it gains $1/m$ from the
translation and $r^2/I = 1/(\kappa m)$ from the rotation. $M_t = m/3$ for the puck means its
rim reacts to a tangential push as if it weighed a third of the puck. For two bodies the
rims give way together, so the inverse masses add. For a body that is not round, the
normal push would also spin it and the two directions would couple.

**Back from the contact to the bodies.** $\Delta u_n$ and $\Delta u_t$ give the impulse,
$P_n = M_n \Delta u_n$ and $P_t = M_t \Delta u_t$, and the impulse gives every velocity
change above. The impulse has two components but changes three puck velocities because it
acts at a known point: it exerts no torque about the contact point, so the puck's angular
momentum about that point is conserved,

```math
\Delta v_t + \kappa r\, \Delta\omega = 0 .
```

For a hit the six velocity changes of the two bodies are fixed by the two impulse
components, the conservation of total momentum (two equations) and the conservation of
each body's angular momentum about the contact point (two equations).

A model is only a choice of $P_n$ and $P_t$.

## 2. Old models

**Wall (planner, `elastic` simulator model).** Normal restitution only, spin ignored:

```math
P_n = -m (1 + e_w)\, u_n, \qquad P_t = 0
\quad\Longrightarrow\quad v_n' = -e_w v_n, \quad v_t' = v_t .
```

The planner uses $e_w = 0.9$ (`table.restitution`).

**Hit (planner `model.impact`).** Restitution $e$ and Coulomb friction $\mu$, no tangential
restitution, spins left out of the slip:

```math
P_n = -M_n (1 + e)\, u_n, \qquad P_t = -\operatorname{clip}\!\left(M_t u_t,\ -\mu P_n,\ \mu P_n\right),
\qquad u_t = (v - v_H)\cdot t .
```

This is Box2D's contact of two disks ($e = 0.8$, $\mu = 0.0632$). The `holohover_dial`
model and the original `puck_collision_viz.py` used the elastic, frictionless hit
$e = 1$, $\mu = 0$.

## 3. New model (Walton)

Three coefficients: normal restitution $e_n$, Coulomb friction $\mu$, tangential restitution
$e_t$. The same law serves the wall and the hit, with the masses of Section 1:

```math
P_n = -(1 + e_n)\, M_n u_n, \qquad
P_t = -\operatorname{sgn}(u_t)\,(1 + e_t)\,\min\!\left(M_t \lvert u_t\rvert,\ \mu P_n\right).
```

Read the tangential impulse from the inside out:

- $M_t \lvert u_t\rvert$ is the impulse that stops the slip (the rim would then roll on the
  other surface),
- $\mu P_n$ is the most friction can give (Coulomb),
- $1 + e_t$: the rim and the wall cushion deform sideways while friction holds the contact
  and spring back at the end, giving back $e_t$ of the friction impulse.

This gives two regimes:

| regime | when | result |
|---|---|---|
| stick | $M_t \lvert u_t\rvert \le \mu P_n$ (little slip, hard hit) | $u_t' = -e_t u_t$: the slip comes back reversed |
| slide | $M_t \lvert u_t\rvert > \mu P_n$ (glancing hit) | $\lvert\Delta u_t\rvert = (1 + e_t)\, \mu P_n / M_t$: friction saturates |

The normal part always gives $u_n' = -e_n u_n$. The old models are special cases: $\mu = 0$
is the elastic model, $e_t = 0$ with the spins dropped is the planner's hit.

**Wall, as velocity changes** ($\kappa = 1/2$, $v_n = v \cdot n < 0$ before the impact):

```math
v_n' = -e_n v_n, \qquad \Delta v_t = \frac{\Delta u_t}{3}, \qquad r\,\Delta\omega = -\frac{2\,\Delta u_t}{3}.
```

In the slide regime the spin drops out:

```math
\Delta v_t = -\operatorname{sgn}(u_t)\; c\, \lvert v_n\rvert, \qquad c = \mu (1 + e_t)(1 + e_n),
```

that is Coulomb's law with the apparent friction coefficient $\mu(1 + e_t)$. A wall impact
always conserves $v_t + \kappa r \omega$ (Section 1): the wall trades speed along it for
spin, and the regime decides how far.

**Coefficients.** The wall slides iff $\lvert u_t\rvert / \lvert u_n\rvert > \lambda$, with
$\lambda = \mu (1 + e_n)(1 + \kappa)/\kappa$.

| contact | $e_n$ | $\mu$ | $e_t$ | $\mu(1+e_t)$ | $c$ | $\lambda$ |
|---|---|---|---|---|---|---|
| wall (fit of `puck_id_2026-10-13_18-15-39`) | 0.884 | 0.070 | 0.458 | 0.102 | 0.192 | 0.396 |
| hovercraft (simulator guess, not identified) | 0.85 | 0.20 | 0.30 | 0.26 | – | – |

$\lambda = 0.396$ is a hit about 22° from perpendicular for a puck that does not spin.

This is Walton's model (O. R. Walton, in *Particulate Two-Phase Flow*, 1993; stated and
measured in S. F. Foerster et al., *Physics of Fluids* 6, 1108, 1994,
[doi:10.1063/1.868282](https://doi.org/10.1063/1.868282)), with Foerster's friction
coefficient equal to $\mu(1 + e_t)$ and his $\beta_0 = e_t$. It is not the independent
friction-restitution model of Doménech-Carbó (2023), which the simulator and
identification docs cite.

Implemented in `holohover_simulator/identification/model.py` (`impact`),
`holohover_simulator/src/simulator_collisions.hpp` (the hovercraft as a moving wall) and
`puck_collision_viz.py` (both contacts, two-body masses).

## 4. Mirror trick and its shear

Planner frame: $x$ towards the opponent goal, side walls at $y = \pm w$ (puck centre), goal
line $x = x_g$, goal mouth $\lvert y\rvert \le h_\text{eff}$. A sequence $(s, k)$ makes $k$
side-wall bounces, the first on wall $s$ ($s = +1$: $y = +w$, $s = -1$: $y = -w$). The puck
leaves the hit at $(x_c, y_c)$ with velocity $v^+ = (a, s b)$, $a, b > 0$.

The drag $\dot v = -\gamma v$ is the same on both axes, so it only changes the speed along a
path: between bounces the puck moves on straight lines.

**Old trick.** A bounce maps $(v_x, v_y) \mapsto (v_x, -e_w v_y)$. Reflect the table about the
wall and stretch the copy by $1/e_w$ across it: in the copy the puck continues on the same
straight line. After $k$ bounces the goal's image is

```math
\bar x = x_g, \qquad
\bar y = s\, w \left(1 + \sum_{j=1}^{k-1} \frac{2}{e_w^{\,j}} + \frac{1}{e_w^{\,k}}\right), \qquad
\bar h = \frac{h_\text{eff}}{e_w^{\,k}},
```

and a shot scores iff the straight line from $(x_c, y_c)$ along $v^+$ passes through it.

**New wall model.** While a bounce slides with the rim slipping forward (Section 5), it maps

```math
(v_x, v_y) \mapsto (v_x - c\,\lvert v_y\rvert,\ -e_n v_y),
```

still linear and independent of the spin. The copy past a wall is now reflected,
stretched by $1/e_n$ and sheared: a point at distance $d$ from the wall moves forward by
$c d / e_n$. The shear is anchored on the wall line, not on the impact point, so the copies
do not depend on the shot and the image of the goal is again fixed. For an aim at height
$y_g$ in the mouth, with $D_j = 2w$ for $j < k$ and $D_k = w - s(-1)^{k-1} y_g$ the
distances in $y$ covered after each bounce:

```math
\bar x = x_g + c \sum_{j=1}^{k} D_j \,\frac{1 + e_n + \dots + e_n^{\,j-1}}{e_n^{\,j}}, \qquad
\bar y = s \left(w + \sum_{j=1}^{k-1} \frac{2w}{e_n^{\,j}} + \frac{D_k}{e_n^{\,k}}\right).
```

$\bar y$ is unchanged. The image moves away along $x$ (by about 0.10 m for one bounce and
0.43 m for two, aiming at the centre) and the mouth becomes a tilted segment. With the
fitted coefficients, shots aimed at the old images miss by 4 to 12 cm after one bounce
($h_\text{eff} = 12.5$ cm) and by 9 to 39 cm after two; shots aimed at the sheared images
land on the aim point exactly.

In the planner: $\bar x$ replaces $x_g$ in the aim, reach and $T_B$ terms, and the aim
condition tests the tilted segment. It stays linear in $v^+$.

`puck_collision_viz.py` draws the sheared copies ("shear the copies"), and the unfolded
trace stays straight as long as every bounce mark is white.

## 5. Constraints that keep the trick exact

The shear holds while every side-wall bounce **slides with the rim slipping forward**,
along $+x$. Two regimes break it:

- stick (yellow in the visualizer): the outgoing velocity depends on the spin, and the copy
  would depend on the impact point;
- slide with the rim slipping backwards (red), because the spin dominates: friction then
  pushes the puck forward and the shear flips sign.

The error grows continuously from zero at the regime boundary, up to about 9 cm for a fully
reversed slip, so soft constraints (with slack) are enough.

Bounce $j$ on wall $s_j$, with velocity $(a_j, \pm b_j)$ and spin $\omega_j$ just before it,
slides forward iff the rim's slip along $+x$ exceeds the stick threshold:

```math
a_j - s_j\, r\, \omega_j \;\ge\; \lambda\, b_j .
```

**First bounce: depends on the spin.** The spin $\omega^+$ comes from the hit (Section 3,
which needs the measured puck spin and the hovercraft's). Between the hit and the wall the
puck's velocity decays by $e^{-\gamma \tau_1} = 1 - \gamma (w - s y_c)/b$ and its spin
decays too. Ignoring the spin decay is conservative when the spin is adverse:

```math
(a - \lambda b)\left(1 - \gamma\,\frac{w - s\,y_c}{b}\right) \;\ge\; r \max(0,\ s\,\omega^+).
```

**Later bounces: a cone on the shot direction.** The bounce map gives
$a_j = a - c b (1 + e_n + \dots + e_n^{\,j-2})$ and $b_j = e_n^{\,j-1} b$. Leaving the spin
out:

```math
a \;\ge\; \beta_j\, b, \qquad \beta_j = c\,(1 + e_n + \dots + e_n^{\,j-2}) + \lambda\, e_n^{\,j-1},
\qquad \beta_1 = 0.396,\ \beta_2 = 0.542 .
```

For the second bounce this is conservative: the spin left by the first bounce adds to the
forward slip at the opposite wall. From the third bounce on, the spins of consecutive
bounces roughly cancel and steep shots drift towards sticking (the slip ratio shrinks
whenever $a_j / b_j < c/(1 - e_n) \approx 1.66$), so $k > 2$ needs a margin on $\beta_j$.

$a \ge \beta_j b$ also keeps $v_x > 0$ up to the goal.

The hit itself needs no regime constraint: the planner models both of its regimes.
