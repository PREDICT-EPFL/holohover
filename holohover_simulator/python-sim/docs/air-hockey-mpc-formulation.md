# Air Hockey MPC Formulation

Bilevel shot planner for the Holohover air hockey setup. The outer loop makes a discrete choice of the wall-bounce sequence; the inner loop is a smooth NLP solved with a classical gradient-based solver (CasADi + IPOPT, later acados). It reuses ideas from Till Beyer's DIAL-MPC project (planner/tracker split, free impact time) and Karlym Nam's classical MPC project (IPOPT, impact model, delay compensation).

Version of 2026-09-28, after review. The review decisions in Section 0 are binding for the first implementation. Section 13 is written for the implementer.

---

## 0 Decisions for the first implementation

| Topic | Decision |
| --- | --- |
| Target simulator | Till's Python simulator first: walls reflect perfectly, robot–puck collisions are elastic, viscous friction $\gamma$ acts on every body. Karlym's Python simulator, ROS Box2D and hardware come later. |
| Frame | $x$ points from our goal to the opponent goal. Wall L is at $y=+W/2$, wall R at $y=-W/2$. Viewed in the simulator with $x$ pointing left, $+y$ points down (confirmed). |
| Pre-impact puck bounces | Not in v1: restrict $t_f \le \tau_1$. They become mandatory soon, because the puck bounces a lot before we can reach it. The code must allow adding mode $m$ (Section 7) without redesign. |
| Max post-impact bounces | $K = 2$, so $\mathbb{B}_2 = \{\varnothing, L, R, LR, RL\}$. |
| Puck mass | $m_p = 10$ g by default, as a configurable parameter. |
| Own-half rule | The whole robot stays behind the midline: $x_k \le x_\text{mid} - r_R$ at every node. |
| Contact-angle cost | Include the optional cost on the angle between $n$ and $v_p^+$ (Section 4). |
| Corner margin | Not a constraint. Geometry already keeps bounces away from the corners (Section 5); keep a runtime check only. |
| Wall model | Smooth sliding model with no stick–slip: air hockey walls do not stick. |
| Cost scaling | Normalise every cost term by a reference value (Section 8). |
| Physical parameters | Read masses, radii, inertia, friction, table size, goal width and input limits from the existing code base. Do not hard-code them. |

---

## 1 Overview and architecture

A strike is planned by two nested minimisations: the outer loop picks the wall-bounce sequence $\mathcal{B}$, the inner loop plans the robot motion for that fixed $\mathcal{B}$.

$$
\min_{\mathcal{B} \in \mathbb{B}_K} \; \min_{x,\,u,\,t_f,\,\varphi,\,\Delta} \; J(x, u, t_f, \varphi, \Delta; \mathcal{B})
$$

Here $x, u$ are the robot states and accelerations, $t_f$ the impact time, $\varphi$ the angle of the contact normal and $\Delta$ the post-impact flight variables. $\mathbb{B}_K$ is the finite set of bounce sequences with at most $K$ side-wall bounces.

Fixing $\mathcal{B}$ removes all switching from the post-impact puck flight, so the inner problem is smooth and a gradient solver applies. The combinatorial part is a finite enumeration: 5 sequences for $K = 2$. The outer minimisation is therefore exact up to the quality of the inner solves.

| Level | Problem | Suggested rate | Output |
| --- | --- | --- | --- |
| Outer: sequence selection | Solve $P(\mathcal{B})$ for every $\mathcal{B} \in \mathbb{B}_K$, keep the cheapest (Section 9) | 5–10 Hz | $\mathcal{B}^\star$ |
| Inner: shot planner | Smooth NLP $P(\mathcal{B})$: coarse post-impact model first, then fine model (Sections 5, 6, 8) | 10–25 Hz | Robot trajectory, $t_f$, contact normal |
| Tracker | Linear tracking MPC, a QP (Section 11) | 50–100 Hz | Command $(a_x, a_y, a_\theta)$ |

What is reused from the two earlier projects:

- From Till: the planner/tracker split, a free impact time $t_f$ with step $\Delta t = t_f / N$, the bounce sequence as a shot label, and the terminal-cost structure (Section 8 maps his terms).
- From Karlym: a classical solver (CasADi + IPOPT), the impulse-based impact model, delay compensation and heading regularisation. His future-work section asks for exactly this planner/tracker architecture.
- What changes: no sampling and no tanh gates. The non-smoothness Till handled by sampling moves into the outer enumeration, and the contact time and place become explicit decision variables.

---

## 2 Models and notation

The robot is a 3-DOF double integrator driven by accelerations. The puck is not optimised; it is predicted in closed form.

### Table frame

- Our end wall is at $x = 0$, the opponent end wall at $x = X$ (table length). The midline is $x_\text{mid} = X/2$.
- The side walls are L at $y = +W/2$ and R at $y = -W/2$.
- Effective walls for the puck centre: $y = \pm w$ with $w = W/2 - r_p$, and goal line $x_g = X - r_p$.
- Goal centre $(x_g, y_g)$, with $y_g = 0$ on a symmetric table. Goal half-width $h$; usable half-mouth $h' = h - r_p - \text{margin}$.
- The simulator frame may differ (Section 13): convert with one transform function.

### Robot

State and input follow the handwritten notes, with viscous friction $\Gamma$:

$$
x_R = (x,\, y,\, \theta,\, v_x,\, v_y,\, v_\theta)^\top, \qquad u = (a_x,\, a_y,\, a_\theta)^\top
$$

$$
\dot x_R = A_R\, x_R + B_R\, u, \qquad
A_R = \begin{pmatrix} 0 & I_3 \\ 0 & -\Gamma \end{pmatrix}, \qquad
B_R = \begin{pmatrix} 0 \\ I_3 \end{pmatrix}
$$

- $\Gamma = \gamma I_3$ matches Till's simulator ($\gamma = 0.3$ in his configuration; read the value from the code). $\Gamma = 0$ is the handwritten model.
- Till's simulator takes forces and a torque, so $a_x = F_x / m_R$, $a_y = F_y / m_R$, $a_\theta = \tau / I_R$, and the input bounds convert the same way.
- The heading $\theta$ does not enter the impact because the Holohover is round. It stays in the state with a regularisation cost, since Karlym's hardware tests showed heading drift coupling into position.

### Puck before impact

The puck is autonomous: $u$ does not affect it until $t_f$. With linear drag $\gamma_p$ its prediction from the measured state $(p_{p,0}, v_{p,0})$ is

$$
p_p(t) = p_{p,0} + v_{p,0}\,\frac{1 - e^{-\gamma_p t}}{\gamma_p}, \qquad v_p(t) = v_{p,0}\, e^{-\gamma_p t}
$$

For $\gamma_p \to 0$ this is $p_{p,0} + v_{p,0}\, t$. The prediction is valid until the puck's next wall hit $\tau_1$. For a wall at coordinate $c$ along axis $i \in \{x, y\}$, lying in the direction of motion:

$$
\tau^{(i)} = -\frac{1}{\gamma_p} \ln\!\Big(1 - \frac{\gamma_p\,(c - p_{p,0,i})}{v_{p,0,i}}\Big), \qquad \tau_1 = \min_{\text{walls ahead}} \tau^{(i)}
$$

If the argument of the logarithm is not positive, the puck stops before reaching that wall. After $\tau_1$ the puck has bounced; Section 7 extends the prediction to that case.

### Changes to the handwritten model

- The puck leaves the optimisation state. Its 6 states only carried a known trajectory, so the closed-form prediction replaces them. $\theta_p$ and $v_{\theta p}$ are dropped: spin plays no role in the frictionless impact model.
- The linear model has no walls, so pre-impact bounces are invisible to it. v1 requires $t_f \le \tau_1$; Section 7 adds the mode $m$.
- The letter $B$ is used twice in the notes. Here $B_R$ is the input matrix and $\mathcal{B}$ the bounce sequence.
- The notes write $\mathcal{B}$ as a set such as $\{R, L\}$. It is an ordered sequence, and side-wall bounces must alternate (Section 5), so $\mathcal{B}$ is fixed by its first wall and its number of bounces.

---

## 3 Free final time and discretisation

Scaling time by $t_f$ fixes the horizon to $[0, 1]$, so $t_f$ becomes an ordinary decision variable that multiplies the dynamics. This is the "move $t_f$ out of the integral bound" trick from the handwritten notes.

### Time scaling

Set $t = t_f\,\tau$ with $\tau \in [0, 1]$, and write $\tilde x(\tau) = x(t_f \tau)$, $\tilde u(\tau) = u(t_f \tau)$. The chain rule and $dt = t_f\, d\tau$ give

$$
\frac{d\tilde x}{d\tau} = t_f\, \big(A_R\, \tilde x + B_R\, \tilde u\big), \qquad
\int_0^{t_f} \ell\big(x(t), u(t)\big)\, dt = t_f \int_0^1 \ell\big(\tilde x(\tau), \tilde u(\tau)\big)\, d\tau
$$

The integration bound no longer depends on a decision variable; $t_f$ only appears as a factor.

### Discretisation (zero-order hold)

Split $[0, 1]$ into $N$ equal intervals, each lasting $\Delta t = t_f / N$ in real time, and hold $u$ constant on each. The exact discretisation is

$$
x_{k+1} = e^{A_R \Delta t}\, x_k + \Big(\int_0^{\Delta t} e^{A_R s}\, ds\Big) B_R\, u_k
$$

For $\Gamma = 0$, $A_R$ is nilpotent ($A_R^2 = 0$), so the exponential series stops after the linear term:

$$
x_{k+1} =
\underbrace{\begin{pmatrix} I_3 & \Delta t\, I_3 \\ 0 & I_3 \end{pmatrix}}_{A_d(\Delta t)} x_k
+ \underbrace{\begin{pmatrix} \tfrac12 \Delta t^2\, I_3 \\ \Delta t\, I_3 \end{pmatrix}}_{B_d(\Delta t)} u_k,
\qquad \Delta t = \frac{t_f}{N}
$$

For $\Gamma = \gamma I_3$ (Till's simulator), solving $\dot v = -\gamma v + u$ and then $\dot p = v$ over one interval gives

$$
A_d = \begin{pmatrix} I_3 & \dfrac{1 - e^{-\gamma \Delta t}}{\gamma}\, I_3 \\ 0 & e^{-\gamma \Delta t}\, I_3 \end{pmatrix}, \qquad
B_d = \begin{pmatrix} \dfrac{1}{\gamma}\Big(\Delta t - \dfrac{1 - e^{-\gamma \Delta t}}{\gamma}\Big) I_3 \\ \dfrac{1 - e^{-\gamma \Delta t}}{\gamma}\, I_3 \end{pmatrix}
$$

### Cost

With a zero-order hold the input part of the running cost is integrated exactly, because $\tilde u$ is constant on each interval of length $1/N$:

$$
t_f \int_0^1 \tilde u^\top R\, \tilde u \, d\tau = \frac{t_f}{N} \sum_{k=0}^{N-1} u_k^\top R\, u_k
$$

State terms (heading, angular rate) use the same rectangle rule at the nodes.

### Pay attention

- The dynamics are no longer linear in the decision variables: $\Delta t\, v_k$ and $\Delta t^2 u_k$ are products of unknowns. Even for a fixed $\mathcal{B}$ the inner problem is an NLP, not a QP, but it stays smooth and small.
- Bound $t_f$ from both sides. $t_{\min} > 0$ avoids a degenerate $\Delta t \to 0$; $t_{\max}$ caps the grid spacing ($N = 20$ and $t_f \le 1$ s give $\Delta t \le 50$ ms).
- Path constraints only hold at the $N + 1$ nodes. On a coarse grid the robot can clip the puck between nodes, so keep a margin $\delta$ (Section 8) or also check midpoints.
- A zero-order hold is a reasonable start. Till's chattering with direct ZOH came from sampling noise, which a gradient solver does not have. If inputs still jump, add a $\Delta u$ penalty, or move to first-order hold or splines later.
- The grid changes at every replan because $t_f$ changes. Warm starts and the tracker must interpolate onto the new grid, not just shift indices.
- Till's simulator integrates with Verlet at 100 Hz; the planner uses the exact ZOH map on a coarser grid. The mismatch is small and the tracker absorbs it.

---

## 4 Robot–puck impact

At $t_f$ the robot touches the puck, and a frictionless impulsive collision along the contact normal $n$ gives the post-impact puck velocity in closed form.

### Contact geometry

Let $n$ be the unit vector from the robot centre to the puck centre, parametrised as $n(\varphi) = (\cos\varphi, \sin\varphi)$ so no norm constraint is needed. With $p_c = p_p(t_f)$ and $v_p^- = v_p(t_f)$ from Section 2, contact at $t_f$ is one equality on the last robot node:

$$
p_{R,N} = p_c - (r_R + r_p)\, n(\varphi)
$$

### Derivation

A frictionless contact transmits an impulse $J n$ along $n$ only. Conservation of momentum for both bodies gives

$$
m_R\, v_R^+ = m_R\, v_R^- - J\, n, \qquad m_p\, v_p^+ = m_p\, v_p^- + J\, n
$$

Define the closing speed $v_n = (v_R^- - v_p^-) \cdot n > 0$. Newton's restitution law with coefficient $e$ sets the separating speed to $e\, v_n$:

$$
(v_p^+ - v_R^+) \cdot n = e\, v_n
$$

Substituting the momentum equations gives $-v_n + J\,(1/m_p + 1/m_R) = e\, v_n$, so $J = (1 + e)\, m_R\, m_p\, v_n / (m_R + m_p)$. Hence

$$
v_p^+ = v_p^- + \kappa\, v_n\, n, \qquad
v_R^+ = v_R^- - \kappa\,\frac{m_p}{m_R}\, v_n\, n, \qquad
\kappa = \frac{(1+e)\, m_R}{m_R + m_p}
$$

For $e = 1$ this is Till's eq. (8) and Karlym's eq. (26). With $v_p^- = 0$ it reduces to Karlym's eq. (6). Sanity value for tests: with $m_R = 140$ g, $m_p = 10$ g and $e = 1$, $\kappa \approx 1.87$, so the puck gains almost twice the closing speed along $n$.

### Constraints in the NLP

$$
v_n = (v_{R,N} - v_p^-) \cdot n(\varphi) \ge v_{\min}, \qquad
v_p^+ = v_p^- + \kappa\, v_n\, n(\varphi)
$$

$v_{\min} > 0$ forces a real strike. The contact equality replaces Till's approach and closeness costs (A) and (D) with an exact condition.

### Contact-angle cost (adopted)

The shot direction is very sensitive to $\varphi$ when $v_p^+$ leaves at a large angle from $n$ (grazing contacts, or a fast incoming puck with a large tangential velocity). A small cost on that angle favours robust strikes:

$$
\ell_n = w_n\, \sin^2\angle\big(n, v_p^+\big) = w_n\, \frac{\big(n_x\, v_{p,y}^+ - n_y\, v_{p,x}^+\big)^2}{\lVert v_p^+ \rVert^2}
$$

It is zero when the puck leaves exactly along $n$. The denominator is safe because the scoring constraint enforces $v_{p,x}^+ \ge \varepsilon$ (Section 5).

### Pay attention

- The incoming tangential velocity survives: $v_p^+$ keeps the component of $v_p^-$ perpendicular to $n$, so the shot does not leave along $n$ when the puck moves. Karlym neglected this; here it is exact, because the aim constraint acts on $v_p^+$, not on $n$.
- The model assumes round bodies, centre of mass at the geometric centre, no contact friction and no spin. On hardware Karlym saw a CoM offset and stronger recoil than predicted. Recoil does not change the shot, but it matters for the robot after impact.
- v1 uses $e = 1$ (Till's simulator is elastic) and $m_p = 10$ g (configurable). $m_R$ comes from the simulator; the hardware Holohover weighs 140 g.
- Keep $\varphi$ unconstrained and warm-started. Do not wrap it to $[-\pi, \pi)$ between solves, or the warm start jumps by $2\pi$.

---

## 5 Coarse post-impact model: the virtual goal

If the walls reflect perfectly, a shot with $k$ bounces is a straight line to a mirrored copy of the goal. This coarse model is exact for Till's simulator. It also provides the warm start for the fine model of Section 6.

### Why bounces alternate

A side wall flips $v_y$ and leaves $v_x$ unchanged, and the flight between bounces is straight. After hitting $y = +w$ the puck therefore moves toward $y = -w$, so consecutive bounces alternate between L and R. A sequence is fixed by its first wall $s$ ($+1$ for L, $-1$ for R) and its length $k$:

$$
\mathbb{B}_K = \{\varnothing\} \cup \{(s, k) : s \in \{+1, -1\},\ 1 \le k \le K\}, \qquad \lvert \mathbb{B}_K \rvert = 2K + 1
$$

For $K = 2$ this is $\{\varnothing, L, R, LR, RL\}$.

### Unfolding, step by step

Take a one-bounce shot off wall L:

1. The real path goes from the impact point $p_c$ to a bounce point $b$ on wall L, then to the goal $g$.
2. At $b$ the elastic bounce flips $v_y$: the wall acts like a mirror, so the angle of incidence equals the angle of reflection.
3. Reflect the second leg ($b \to g$) across wall L. It becomes $b \to \bar g$, where $\bar g$ is the mirror image of the goal.
4. Because the angles are equal, the reflected second leg continues the first leg in a straight line: $p_c$, $b$ and $\bar g$ are collinear.
5. Hence "a one-bounce shot off L that scores" is the same as "a straight shot from $p_c$ toward $\bar g$". The bounce point is where that straight line crosses the wall.

```text
y = +3w   ------------------------------------|  image of wall R
                                              |
y = +2w                                       @  virtual goal g_B
                                           /  |
                                        /     |  mirrored copy of the table
                                     /        |
y = +w    ========================*===========|  wall L: bounce point b
                               /     \        |
                            /           \     |  real table
                         /                 \  |
y =  0            p_c o                       @  real goal g
                                              |
y = -w    ====================================|  wall R
                                              x = x_g
```

The straight line from $p_c$ to the virtual goal $g_\mathcal{B}$ is the unfolded shot; the real puck follows it up to $b$, then the mirrored leg down to $g$. For $k$ bounces, repeat the construction after each bounce: the table becomes a stack of mirrored copies, and the goal's image lands in copy number $k$.

### Closed form: where the virtual goal is

Reflecting a point across wall L ($y = +w$) maps $y \mapsto 2w - y$; across wall R ($y = -w$) it maps $y \mapsto -2w - y$. The $x$ coordinate never changes, so $\bar g_\mathcal{B} = (x_g, \bar y_\mathcal{B})$.

To find $\bar y_\mathcal{B}$, reflect the real goal across the walls of $\mathcal{B}$ in reverse order, last wall first. Example $\mathcal{B} = LR$ (first L, then R):

$$
y_g \;\xrightarrow{\ \text{reflect across R}\ }\; -2w - y_g \;\xrightarrow{\ \text{reflect across L}\ }\; 2w - (-2w - y_g) = 4w + y_g
$$

Each reflection moves the goal one table width ($2w$) further out on the side of the first wall and flips the sign of the goal offset $y_g$. This gives the general formula

$$
\bar y_\mathcal{B} = 2\, s\, k\, w + (-1)^k\, y_g
$$

| $\mathcal{B}$ | $s$ | $k$ | $\bar y_\mathcal{B}$ | with $y_g = 0$ |
| --- | --- | --- | --- | --- |
| $\varnothing$ (direct) | – | 0 | $y_g$ | $0$ |
| L | $+1$ | 1 | $2w - y_g$ | $2w$ |
| R | $-1$ | 1 | $-2w - y_g$ | $-2w$ |
| LR | $+1$ | 2 | $4w + y_g$ | $4w$ |
| RL | $-1$ | 2 | $-4w + y_g$ | $-4w$ |

Proof by induction: the formula holds for $k = 0$. If $\mathcal{B}$ starts with wall $s$ and the remaining $k-1$ bounces $\mathcal{B}'$ start with $-s$, then $\bar y_{\mathcal{B}'} = -2s(k-1)w + (-1)^{k-1} y_g$, and reflecting across the first wall gives $\bar y_\mathcal{B} = 2sw - \bar y_{\mathcal{B}'} = 2skw + (-1)^k y_g$.

### The bounce order comes for free

The straight line from $p_c$ (with $\lvert y_c \rvert < w$) to $\bar y_\mathcal{B}$ crosses $y = sw, 3sw, \dots, (2k-1)sw$: exactly $k$ wall images, alternating between the real walls and starting with wall $s$. Since $x$ increases monotonically to $x_g$, every crossing happens before the goal line. The image of the mouth, $[\bar y_\mathcal{B} - h', \bar y_\mathcal{B} + h']$, stays inside copy number $k$ as long as $h' < w$. So aiming the unfolded shot at the image of the mouth enforces the whole sequence $\mathcal{B}$.

### Scoring constraint

The unfolded shot crosses $x = x_g$ at

$$
y_\text{hit} = y_c + (x_g - x_c)\, \frac{v_{p,y}^+}{v_{p,x}^+}
$$

Requiring $\lvert y_\text{hit} - \bar y_\mathcal{B} \rvert \le h'$ and multiplying by $v_{p,x}^+ > 0$ removes the division:

$$
\big\lvert\, (y_c - \bar y_\mathcal{B})\, v_{p,x}^+ + (x_g - x_c)\, v_{p,y}^+ \,\big\rvert \le h'\, v_{p,x}^+, \qquad v_{p,x}^+ \ge \varepsilon
$$

These are two smooth inequalities, linear in $v_p^+$ for a given $p_c$. The aim error used in the cost is $e_\text{aim} = y_\text{hit} - \bar y_\mathcal{B}$.

### Time to goal, step by step

1. After impact the puck obeys $\dot v = -\gamma_p v$, so $v(t) = v_p^+ e^{-\gamma_p t}$: the direction stays constant and only the speed decays.
2. The distance travelled after time $t$ is the integral of the speed:

$$
s(t) = \int_0^t \lVert v_p^+ \rVert\, e^{-\gamma_p t'}\, dt' = \lVert v_p^+ \rVert\, \frac{1 - e^{-\gamma_p t}}{\gamma_p}
$$

3. An elastic bounce flips $v_y$ but keeps the speed, and reflections preserve lengths. The real path length therefore equals the unfolded straight length $L_\mathcal{B} = \lVert (x_g, y_\text{hit}) - p_c \rVert$, and $s(t)$ still applies across bounces.
4. Setting $s(T_\mathcal{B}) = L_\mathcal{B}$ and solving for $T_\mathcal{B}$:

$$
T_\mathcal{B} = -\frac{1}{\gamma_p} \ln\!\Big(1 - \frac{\gamma_p\, L_\mathcal{B}}{\lVert v_p^+ \rVert}\Big)
$$

5. The puck can travel at most $\lVert v_p^+ \rVert / \gamma_p$, so the logarithm is only defined if it reaches the goal. The speed at the goal is $\lVert v_p^+ \rVert e^{-\gamma_p T_\mathcal{B}}$; requiring at least a fraction $\eta$ of the launch speed gives the reach constraint

$$
\gamma_p\, L_\mathcal{B} \le (1 - \eta)\, \lVert v_p^+ \rVert
$$

For $\gamma_p \to 0$, $T_\mathcal{B} = L_\mathcal{B} / \lVert v_p^+ \rVert$.

### Bounce points for the warm start

The $j$-th crossing sits at the fraction $\lambda_j = ((2j - 1)sw - y_c) / (y_\text{hit} - y_c)$ of the unfolded shot. Its real position is $x_j = x_c + \lambda_j (x_g - x_c)$, $y_j = s(-1)^{j-1} w$, and its time follows from the formula of step 4 with distance $\lambda_j L_\mathcal{B}$.

### Pay attention

- Use the effective wall $w = W/2 - r_p$ and goal line $x_g = X - r_p$, not the table dimensions.
- The model is exact only for elastic, frictionless walls. That is the case in Till's simulator; Karlym's simulator uses a wall restitution of 0.8, which bends the unfolded path (Section 6).
- Each bounce lengthens the path and amplifies angle errors. $T_\mathcal{B}$ already charges the extra time; the outer penalty $w_b \lvert \mathcal{B} \rvert$ (Section 9) charges the extra risk.

---

## 6 Fine post-impact model: the bounce chain

For finer planning the bounce points become decision variables, and the flight is chained segment by segment through any smooth wall model $W_b$. With $\mathcal{B}$ fixed the NLP knows which wall each bounce hits, so every constraint stays smooth, as in legged-robot trajectory optimisation with a fixed contact schedule.

### Variables

For $\mathcal{B} = (b_1, \dots, b_k)$: segment durations $\Delta_1, \dots, \Delta_{k+1} \ge 0$, plus bounce positions $q_j$ and outgoing velocities $v_j$ for $j = 1, \dots, k$. The chain starts at the impact, $(q_0, v_0) = (p_c, v_p^+)$. That is at most $5k + 1$ extra variables, 11 for $k = 2$.

### Flight map

With linear drag, as in Section 2:

$$
q_{j+1} = q_j + v_j\, \frac{1 - e^{-\gamma_p \Delta_{j+1}}}{\gamma_p}, \qquad
v_{j+1}^- = v_j\, e^{-\gamma_p \Delta_{j+1}}, \qquad j = 0, \dots, k
$$

### Bounce $j$

On wall $b_j$ with outward normal $\nu_j$ ($\nu = (0, 1)$ for L, $(0, -1)$ for R):

$$
\nu_j^\top q_j = w, \qquad \nu_j^\top v_j^- \ge \varepsilon, \qquad v_j = W_{b_j}\big(v_j^-\big)
$$

The first condition puts the puck on the wall, the second makes it arrive moving into the wall, and the third applies the wall model.

### Goal

The end point $q_{k+1}$ given by the flight map lies on the goal line inside the mouth, and every segment moves toward the goal:

$$
x_{k+1} = x_g, \qquad \lvert y_{k+1} - y_g \rvert \le h', \qquad v_{x,j} \ge \varepsilon \ \ (j = 0, \dots, k), \qquad
T_\mathcal{B} = \sum_{j=1}^{k+1} \Delta_j \le -\frac{\ln \eta}{\gamma_p}
$$

The time to goal is now exact and replaces the logarithm formula of Section 5. The bound on $T_\mathcal{B}$ is the reach margin of Section 5 in chain form. The aim error is $e_\text{aim} = y_{k+1} - y_g$.

### Wall maps

Split the velocity into normal and tangential parts, $v = (\nu^\top v)\,\nu + (I - \nu\nu^\top)\, v$.

| Wall model | $W_b(v)$ | Where it fits |
| --- | --- | --- |
| Pure reflection | $(I - 2\nu\nu^\top)\, v$ | Till's simulator (v1); recovers Section 5 exactly |
| Restitution $e_w$, tangential loss $\mu_w$ | $(1 - \mu_w)(I - \nu\nu^\top)\, v - e_w\, \nu\nu^\top v$ | Karlym's Python simulator ($e_w = 0.8$, $\mu_w = 0$) |
| Fitted model | Smooth regression $W_\theta(v)$, e.g. polynomial or small MLP | ROS Box2D or hardware logs |

Air hockey walls do not stick, so a sliding model is enough; $\mu_w$ is small or zero.

### Why the sequence stays consistent

A straight segment from one side wall to the other cannot touch a third side wall, and $v_{x,j} \ge \varepsilon$ rules out the end walls before $x_g$. The only requirement on $W$ is that it never flips $v_x$; the per-segment constraint catches a fitted model that does.

### Warm start from the coarse model

Take $\Delta_j$, $q_j$ and $v_j$ from the virtual-goal line of Section 5, reflecting the velocity at each crossing. With restitution the true shot needs a steeper launch angle, because $v_y$ shrinks at each bounce and the puck drifts further in $x$ per crossing. The warm start is then slightly infeasible, which IPOPT handles well.

The chain could be condensed to the $k + 1$ durations only. Keeping $q_j$ and $v_j$ explicit is the multiple-shooting choice and is usually better conditioned.

### Pay attention

- In v1 the wall map is pure reflection, so the fine model gives the same shots as the coarse one. Implement it anyway: it is the path to Karlym's simulator, ROS and hardware, and the agreement is a useful test.
- Identify $e_w$ and $\mu_w$, or fit $W$, on each new simulator. Planning with pure reflection against a 0.8 wall misplaces the shot after the first bounce.
- Without the reach bound, a slow puck pushes $\Delta$ toward infinity, where the flight map flattens and gradients vanish.
- Model error compounds with $k$. Shrink $h'$ as $k$ grows, and keep $K = 2$ until the wall model is validated.

---

## 7 Pre-impact puck bounces (mode $m$)

The puck often bounces before the robot can reach it, so the planner must be able to strike after $m$ pre-impact bounces. v1 uses $m = 0$ only ($t_f \le \tau_1$), but this is required soon, so the code should support it from the start.

### Event-based prediction

Once per replan, forward-simulate the autonomous puck with the same wall model $W$ up to $t_{\max}$. This gives a list of events $(\tau_m, q_m, v_m)$, $m = 0, \dots, M$, where $\tau_0 = 0$, $(q_0, v_0)$ is the measured puck state, and $(q_m, v_m)$ is the state just after the $m$-th wall hit. The hit times follow from the $\tau^{(i)}$ formula of Section 2 applied to each segment. Stop at a goal (the puck enters a mouth) or at $t_{\max}$.

Within window $m$ the puck is in free flight, so

$$
p_p(t) = q_m + v_m\, \frac{1 - e^{-\gamma_p (t - \tau_m)}}{\gamma_p}, \qquad
v_p(t) = v_m\, e^{-\gamma_p (t - \tau_m)}, \qquad t \in [\tau_m, \tau_{m+1}]
$$

This is smooth in $t_f$ inside a window and works for any wall model, because the events are computed numerically, not optimised. With pure reflection it is the folded straight line.

### Mode in the outer loop

For mode $m$, constrain the impact to its window:

$$
\tau_m + \varepsilon_t \le t_f \le \tau_{m+1} - \varepsilon_t
$$

The outer loop then enumerates pairs $(m, \mathcal{B})$, giving $(M + 1)(2K + 1)$ problems. Skip windows the robot cannot reach in time and windows where the puck stays in the opponent half.

### Pay attention

- Puck avoidance at node $k$ uses $p_p(k\, t_f / N)$, and early nodes may fall in earlier windows. Because $t_f$ moves during the solve, a node can cross a window boundary, which is a kink. Freeze each node's window from the initial guess for the duration of one solve, and re-assign at the next replan.
- An impact right at a puck wall bounce ($t_f$ near $\tau_m$) puts the puck against the wall and the robot next to it. The margin $\varepsilon_t$ and the table constraints handle this.
- Prediction error grows with $m$ (velocity noise is reflected at each bounce). Consider shrinking $h'$ or adding a penalty $w_m\, m$, like $w_b$ for post-impact bounces.

---

## 8 Inner problem $P(\mathcal{B})$

For a fixed $\mathcal{B}$ (and mode $m$) the inner problem is a smooth NLP of about 200 variables that returns the robot trajectory, the impact time and the contact normal.

### Decision variables

$z = (x_0, \dots, x_N,\ u_0, \dots, u_{N-1},\ t_f,\ \varphi,\ \text{post-impact variables})$. The coarse model (Section 5) has no post-impact variables; the fine model adds $(\Delta_j, q_j, v_j)$ (Section 6). Below, $p_k = (x_k, y_k)$ is the robot position at node $k$ and $u_{-1}$ the last applied command.

### Cost

Every term is normalised by a reference value, so the weights are dimensionless and start at order 1:

$$
\begin{aligned}
J(z; \mathcal{B}) = {} & \frac{t_f}{N} \sum_{k=0}^{N-1} \Big( \bar u_k^\top R\, \bar u_k + w_\theta\, \big(\theta_k - \theta_\text{ref}\big)^2 + w_\omega\, v_{\theta,k}^2 \Big)
+ \sum_{k=0}^{N-1} \big\lVert \bar u_k - \bar u_{k-1} \big\rVert_{R_\Delta}^2 \\
& + w_t\, \frac{t_f}{t_\text{ref}} + w_T\, \frac{T_\mathcal{B}}{T_\text{ref}} + w_a \Big(\frac{e_\text{aim}}{h'}\Big)^2 + w_n\, \sin^2\angle\big(n, v_p^+\big)
\end{aligned}
$$

- $\bar u_k = u_k / u_{\max}$ componentwise.
- $e_\text{aim}$ is the offset of the goal-line crossing: $y_\text{hit} - \bar y_\mathcal{B}$ in the coarse model, $y_{k+1} - y_g$ in the fine one.
- Starting values: $t_\text{ref} = 0.5$ s (typical impact time), $T_\text{ref} = 1$ s (typical flight time). Tune from logs.

### Constraints

$$
\begin{aligned}
& x_0 = \hat x_R && \text{measured, delay-compensated state} \\
& x_{k+1} = A_d(t_f/N)\, x_k + B_d(t_f/N)\, u_k && k = 0, \dots, N-1 \\
& \lvert u_k \rvert \le u_{\max} && \text{componentwise} \\
& r_R \le x_k \le x_\text{mid} - r_R, \quad \lvert y_k \rvert \le W/2 - r_R && k = 0, \dots, N, \ \text{whole robot in own half, soft} \\
& \lVert p_k - p_p(k\, t_f / N) \rVert^2 \ge (r_R + r_p + \delta)^2 && k = 0, \dots, N-1, \ \text{soft} \\
& t_{\min} \le t_f \le \min(t_{\max}, \tau_1) && \text{v1; window of Section 7 for } m > 0 \\
& \text{impact (Section 4)} && p_{R,N},\ v_n \ge v_{\min},\ v_p^+ \\
& \text{post-impact (Section 5 or 6)} && \text{scoring, reach, bounces}
\end{aligned}
$$

The own-half constraint at node $N$ also keeps the contact on our side. For scale: if Till's force limit of 0.35 N applies to a 140 g robot, $u_{\max} \approx 2.5$ m/s² per axis.

### Why all states, not only $x_f$

The intermediate states carry the path constraints: table limits, own half, and no contact with the puck before $t_f$. Without path constraints the problem could be condensed, because for a fixed $t_f$

$$
x_N = A_d^N x_0 + \sum_{k=0}^{N-1} A_d^{N-1-k} B_d\, u_k
$$

is affine in $u$, leaving only $(u, t_f, \varphi)$ as variables. Both forms have the same optimum. Keeping every $x_k$ (multiple shooting) gives a larger but sparse and better-conditioned problem, which IPOPT and acados exploit.

### Size

For $N = 20$: 126 states, 60 inputs, $t_f$ and $\varphi$, plus at most 11 post-impact variables, about 200 in total. Karlym solved a problem of similar size ($N = 20$) with CasADi and IPOPT in 3.7 ms on average.

### Mapping to Till's cost terms

| Till term | Here |
| --- | --- |
| (A) approach reward | Contact equality (Section 4) |
| (B) input cost | $\bar u^\top R \bar u$ and the $\Delta u$ penalty |
| (C) impact time | $w_t\, t_f / t_\text{ref}$ |
| (D) closeness at impact | Contact equality (Section 4) |
| (E) goal accuracy | $w_a (e_\text{aim}/h')^2$ plus the hard mouth constraint |
| (F) opponent interception | Dropped for now (Section 12) |
| (G) time to goal | $w_T\, T_\mathcal{B} / T_\text{ref}$, closed form or exact chain |
| (H) validity | Constraints: $v_{p,x}^+ \ge \varepsilon$, mouth, reach, own half |

Karlym's momentum reward is replaced by $T_\mathcal{B}$, which drops as the shot gets faster. His tanh gates disappear because the contact time and place are explicit.

### Pay attention

- The problem is nonconvex: the impact is bilinear, puck avoidance is a reverse-convex constraint, and $t_f$ multiplies the dynamics. Solves are local, so the warm start (Section 10) matters.
- Puck avoidance is needed whenever the robot must go around the puck to get behind it, as in Karlym's stationary-puck test. Without it the plan passes through the puck.
- $w_T$ pushes toward saturated inputs. Balance it with $R$, or penalise the distance to a target shot speed instead.
- Soft constraints need exact (L1) slack penalties, large enough that slack only appears when the hard version is infeasible.

---

## 9 Outer problem: choosing the bounce sequence

The outer loop solves $P(\mathcal{B})$ for every sequence in $\mathbb{B}_K$ and keeps the cheapest, with a switching penalty so the plan does not jump between shots.

$$
\mathcal{B}^\star = \arg\min_{\mathcal{B} \in \mathbb{B}_K} \; J^\star(\mathcal{B}) + w_b\, \lvert \mathcal{B} \rvert + \rho\, \mathbf{1}\big[\mathcal{B} \neq \mathcal{B}_\text{prev}\big]
$$

$J^\star(\mathcal{B})$ is the optimal inner cost, set to $+\infty$ when $P(\mathcal{B})$ is infeasible, and $\lvert \mathcal{B} \rvert = k$ is the number of bounces.

### Procedure at each outer update

1. For each $\mathcal{B} \in \mathbb{B}_K$ (5 for $K = 2$), solve the coarse $P(\mathcal{B})$ from its own warm start. The problems are independent, so run them in parallel.
2. Discard the infeasible ones. If none is feasible, switch to defence (Till's DEF state, Karlym's defend cost) and keep planning.
3. Solve the fine $P(\mathcal{B})$ only for the best one or two candidates, starting from their coarse solutions. In v1 (pure reflection) this step can be skipped.
4. Pick $\mathcal{B}^\star$ with the rule above and pass its plan to the tracker.

With pre-impact modes (Section 7) the set becomes pairs $(m, \mathcal{B})$.

### Choosing the terms

- $w_b \lvert \mathcal{B} \rvert$ is a prior against bounces: each one adds model error and delay. Tune it so a bounce shot wins only when the direct one is blocked or clearly worse.
- $\rho$ is hysteresis. Till saw plans jump between valid shots and tried clustering samples by bounce sequence; it failed because splitting the sample pool raised the variance. Here each $\mathcal{B}$ gets its own deterministic solve, so the comparison is like for like with no variance penalty.
- Near impact ($t_f$ below a commit time, e.g. 150 ms), freeze $\mathcal{B}$ but never $t_f$. Till found that locking $t_f$ dropped conversion from 78 % to 32 % at 50 ms delay.

### Pay attention

- The outer minimisation is only as good as the inner solves: a local minimum in one $P(\mathcal{B})$ can make the wrong sequence win. Per-$\mathcal{B}$ warm starts and a few $t_f$ seeds for cold starts reduce this.
- Treat "not converged" differently from "infeasible". A solve stopped at the iteration cap may still be a good plan; accept it only if it meets the constraints within tolerance.
- Costs of different $\mathcal{B}$ must be comparable: same weights, same normalisation, and $T_\mathcal{B}$ computed the same way for every sequence.
- Compute budget: 5 coarse plus 1–2 fine solves per outer update. At Karlym's 4–6 ms per IPOPT solve that is about 30–40 ms sequentially, so parallelise or lower the outer rate.
- The coarse NLP has the same structure for every $\mathcal{B}$: only $\bar y_\mathcal{B}$ changes. Build one solver with $\bar y_\mathcal{B}$ as a parameter. The fine chain changes structure with $k$, so build one solver per $k$ with the wall normals as parameters.

---

## 10 Warm start and initialisation

Each solve starts from the previous solution of the same $\mathcal{B}$; a cold start uses a closed-form guess built from the virtual goal and an inverted impact model.

### Warm start

Keep one stored solution per $(m, \mathcal{B})$. At the next update set $t_f \leftarrow t_f - \Delta t_\text{replan}$ and interpolate the stored trajectory onto the new grid, since $\Delta t$ changed. Keep $\varphi$ unwrapped.

### Cold start (closed form)

1. Guess $t_f$. Use a few seeds, e.g. 3 values between $t_{\min}$ and $\min(t_{\max}, \tau_1)$, and keep the best solve.
2. Predict the puck at impact: $p_c = p_p(t_f)$, $v_p^- = v_p(t_f)$ (Sections 2 and 7).
3. Aim at the virtual goal: $d = (\bar g_\mathcal{B} - p_c) / \lVert \bar g_\mathcal{B} - p_c \rVert$ (Section 5).
4. Invert the impact for a chosen shot speed $\sigma$ (formula below).
5. Strike state: $p_R = p_c - (r_R + r_p)\, n$, $v_R = (v_p^- \cdot n + v_n)\, n$ (robot moving along $n$ only), $\theta = \theta_\text{ref}$, $v_\theta = 0$.
6. Robot trajectory: the minimum-energy path of a double integrator between two fixed states is a cubic, the Hermite interpolant below. Sample it at the nodes for $x_k$, and take $u_k$ as its second derivative at the interval midpoints.
7. If the cubic passes through the puck, add a via point behind the puck (offset along $-n$), or let the soft avoidance constraint push it out.
8. Post-impact variables: bounce points and times from the virtual-goal line (Section 5).
9. Solve the coarse $P(\mathcal{B})$, then the fine $P(\mathcal{B})$ from its solution.

Impact inversion (step 4). Requiring $v_p^+ = \sigma d$ in Section 4 gives $\kappa\, v_n\, n = \sigma d - v_p^-$, hence

$$
n = \frac{\sigma d - v_p^-}{\lVert \sigma d - v_p^- \rVert}, \qquad
v_n = \frac{\lVert \sigma d - v_p^- \rVert}{\kappa}, \qquad
\varphi = \operatorname{atan2}(n_y, n_x)
$$

This generalises Karlym's eq. (6) to a moving puck, and $v_n > 0$ holds by construction.

Minimum-energy cubic (step 6), per axis, with $s = t / t_f$ and boundary states $(p_0, v_0)$, $(p_f, v_f)$:

$$
p(t) = (2s^3 - 3s^2 + 1)\, p_0 + (s^3 - 2s^2 + s)\, t_f\, v_0 + (3s^2 - 2s^3)\, p_f + (s^3 - s^2)\, t_f\, v_f
$$

### Pay attention

- Pick $\sigma$ from what the robot can reach: $v_R$ must be achievable within $t_f$ under $u_{\max}$. Start with a modest $\sigma$ and let the solver raise it.
- The cubic ignores input limits. If it violates $u_{\max}$ badly, move to a larger $t_f$ seed rather than trusting the solver to repair it.
- Do not warm-start one $\mathcal{B}$ from another's solution. Their optima sit in different basins, which is the reason for the split.

---

## 11 Closed loop: tracking, delay and solvers

The planner runs in closed loop: a fast tracking QP follows the plan, the state is delay-compensated before every solve, and each level replans at its own rate.

### Tracker

Use Till's path-following MPC on the planned states $x_\text{ref}$ and inputs $u_\text{ref}$, with the horizon ending at the planned impact:

$$
\min_{x,u} \sum_{k=0}^{H-1} \lVert x_k - x_{\text{ref},k} \rVert_Q^2 + \lVert u_k - u_{\text{ref},k} \rVert_R^2
\quad \text{s.t.} \quad x_{k+1} = A_d x_k + B_d u_k, \quad \lvert u_k \rvert \le u_{\max}, \quad x_H = x_\text{ref}(t_f)
$$

The robot never bounces off a wall in this formulation, so Till's mirroring trick is not needed. For a given step the problem is a QP; OSQP or HPIPM solve it well under 1 ms at this size.

### Merging planner and tracker

With $\mathcal{B}^\star$ fixed, $P(\mathcal{B}^\star)$ is small enough to run as the MPC itself at 25–50 Hz, applying only the first input. The outer loop then re-selects $\mathcal{B}$ at 5–10 Hz. This removes a layer and the plan–track mismatch, but exposes the loop to IPOPT solve-time jitter. Make it a configuration switch.

### Delay compensation

Before each solve, roll the measured robot state forward through the buffered inputs over the latency $d$ (Karlym), and propagate the puck with its prediction model. Use a conservative upper bound on $d$, as Till did; Karlym used $d = 80$ ms, two 40 ms control periods.

### Game logic

Keep Till's two-state machine: defend at home while planning, attack once a feasible plan exists, and return to defence after contact.

### Solvers

- Prototype: CasADi + IPOPT with Karlym's settings (JIT with `-O3`, shifted warm start, at most 40 iterations, tolerance $10^{-4}$).
- Real time: acados with SQP-RTI (one QP per sample) for the inner problem, plus an iteration cap with fallback to the shifted previous plan.
- The 5 outer problems are independent: one thread each.

### Evaluation

Reuse Till's protocol for comparability: shot conversion over a fixed set of random initial conditions, delays of 0, 50 and 100 ms, and opponents none, static and moving. His DIAL results at 50 ms delay are 74–78 % with no opponent, 50–62 % against a static one and 22–28 % against a moving one.

### Pay attention

- The hard terminal constraint in the tracker can become infeasible after a disturbance. Soften it, or trigger an immediate replan when it fails.
- As $t_f$ shrinks, both horizons shrink. Below a few nodes, stop re-selecting $\mathcal{B}$ and let the tracker finish the last plan.
- Model mismatch shows up here first. On hardware Karlym found heading–position coupling, unmodelled latency and puck velocity noise; keep the heading cost and filter the puck velocity.

---

## 12 Where to pay attention

The formulation is smooth only inside a fixed mode, so most risks sit at mode boundaries, in local minima and in model mismatch.

| Risk | Why it bites | Mitigation | Section |
| --- | --- | --- | --- |
| Pre-impact puck bounces | $p_p(t_f)$ has a kink at each wall hit | v1: $t_f \le \tau_1$; then mode $m$ | 2, 7 |
| Node-to-window assignment ($m > 0$) | Node times move with $t_f$ and can cross a window boundary | Freeze the assignment per solve | 7 |
| Optimum at a mode boundary | Contacts drift toward grazing, $t_f$ toward window edges | Margins $v_{\min}$, $\varepsilon$, $\varepsilon_t$; contact-angle cost; outer loop switches modes | 4–7 |
| Local minima | Bilinear impact, reverse-convex avoidance, $t_f$ scaling | Per-$\mathcal{B}$ warm start, closed-form cold start, several $t_f$ seeds | 10 |
| Constraints only at nodes | The robot can clip the puck between nodes | Margin $\delta$, larger $N$, midpoint checks | 3, 8 |
| Degenerate limits | $t_f \to 0$, $v_{p,x}^+ \to 0$ or $\lVert v_p^+ \rVert \to 0$ make $\Delta t$, $y_\text{hit}$ or $T_\mathcal{B}$ blow up | Bounds $t_{\min}$, $\varepsilon$, $v_{\min}$ and reach margin $\eta$ | 3–6 |
| Wall model mismatch | Pure reflection is wrong on other simulators (restitution 0.8 in Karlym's) | Fine chain with an identified or fitted $W$ | 6 |
| Impact model mismatch | CoM offset, recoil and contact friction on hardware | Identify $e$ and $m_p$; keep an aim margin $h'$ | 4 |
| Execution scatter | Karlym measured contact standard deviations of 35–39 mm on a fast puck | Aim margin $h'$ shrinking with $k$, bounce penalty $w_b$ | 5, 9 |
| Plan switching | Jumping between shots wastes the approach | Hysteresis $\rho$, commit on $\mathcal{B}$ near impact, never lock $t_f$ | 9 |
| Solve-time jitter | IPOPT took 3.7 ms on average and 11.4 ms worst case in Karlym's tests | Iteration cap, fallback plan, acados SQP-RTI | 11 |
| Frame and units | Simulator axes, side of play, forces vs accelerations | One transform function and one unit conversion, both tested | 13 |
| Opponent ignored | A defender can sit on the chosen line | Later: penalise the opponent's distance to the unfolded shot (Till's term F) | 8 |

Open for later simulators: the wall model ($e_w$, $\mu_w$ or a fitted $W$) and the impact restitution $e$ on hardware.

---

## 13 Implementation notes

This section is for the implementer working in the existing code base. Adapt names and structure to what is already there.

### Scope of v1

- Target: Till's Python simulator, with no opponent or a static one first.
- Modes: $m = 0$ only ($t_f \le \tau_1$) and $K = 2$.
- Post-impact: the coarse model in the NLP (exact for pure reflection). Also implement the fine chain with $W$ = pure reflection, plus a test that both give the same shots.
- Architecture: planner + tracker, with a switch to run $P(\mathcal{B}^\star)$ directly as the MPC.
- Solver: CasADi + IPOPT.
- The puck predictor produces the full event list from the start, so enabling $m > 0$ later is a configuration change plus the outer-loop enumeration.

### Check in the code base first

- The simulator's coordinate frame and which side we play. Map it to the planner frame ($x$ toward the opponent goal, wall L at $+y$) with a single `world_to_planner` / `planner_to_world` pair, and test it.
- Whether the simulator takes forces/torque or accelerations. Convert with $m_R$ and $I_R$.
- Masses, radii, inertia, friction per body, table size, goal width, simulator time step, integrator and collision model. Confirm pure wall reflection and $e = 1$.
- The controller interface and the DEF/ATT state machine to plug into, and the opponent controller.
- Existing pieces to reuse, in particular the path-following MPC from Till's project.

### Parameters

Keep every value in one configuration object; nothing hard-coded.

| Parameter | Symbol | Default | Source |
| --- | --- | --- | --- |
| Puck mass | $m_p$ | 0.010 kg | Review decision, configurable |
| Robot mass, inertia | $m_R$, $I_R$ | From simulator | 140 g on hardware |
| Radii | $r_R$, $r_p$ | From simulator | – |
| Table length, width | $X$, $W$ | From simulator | – |
| Goal half-width, margin | $h$, margin | From simulator, 0.01 m | – |
| Viscous friction | $\gamma$, $\gamma_p$ | From simulator | 0.3 1/s in Till's configuration |
| Input limits | $u_{\max}$ | Simulator force/torque limits divided by $m_R$, $I_R$ | 0.35 N, 0.35 N, 0.005 N·m in Till's configuration |
| Impact restitution | $e$ | 1.0 | Elastic in Till's simulator |
| Wall model | $W$ | Pure reflection | Options: reflection, restitution ($e_w$, $\mu_w$), fitted |
| Horizon | $N$, $t_{\min}$, $t_{\max}$ | 20, 0.05 s, 1.0 s | Starting values |
| Bounces, pre-impact modes | $K$, $M$ | 2, 0 | Review decision |
| Margins | $v_{\min}$, $\varepsilon$, $\varepsilon_t$, $\delta$, $\eta$ | 0.2 m/s, 0.05 m/s, 0.02 s, 0.01 m, 0.3 | Starting values, tune |
| Cost references | $t_\text{ref}$, $T_\text{ref}$ | 0.5 s, 1.0 s | Starting values |
| Weights | $R$, $R_\Delta$, $w_\theta$, $w_\omega$, $w_t$, $w_T$, $w_a$, $w_n$, $w_b$, $\rho$ | Order 1 after normalisation | Tune |
| Rates, delay | outer, inner, tracker, $d$ | 5 Hz, 20 Hz, 100 Hz, 50 ms | Starting values |

### Suggested components

- `params`: configuration object and the frame/unit conversions.
- `puck_predictor`: closed-form flight, wall-hit events $(\tau_m, q_m, v_m)$, evaluation in a window.
- `bounce_geometry`: the sequences $\mathbb{B}_K$, the virtual goal $\bar y_\mathcal{B}$, bounce points, time to goal, corner check.
- `impact`: the impact map and its inversion.
- `shot_nlp`: CasADi builder for $P(\mathcal{B})$, coarse and fine variants, built once and reused with parameters ($\hat x_R$, puck window data, $\bar y_\mathcal{B}$ or wall normals, $u_{-1}$).
- `shot_selector`: outer loop, hysteresis, warm-start store per $(m, \mathcal{B})$, cold-start construction.
- `tracker`: the tracking QP.
- `controller`: integration into the simulator loop and state machine, delay compensation, and logging (chosen $\mathcal{B}$, $t_f$, solve times, solver status).

### Tests

Unit tests against the simulator:

1. Discretisation: $A_d$, $B_d$ against the simulator under a constant input (tolerance for Verlet integration).
2. Puck predictor: wall-hit times and positions against a simulator rollout without the robot.
3. Impact map against a simulator collision with given velocities, including the check $\kappa \approx 1.87$ for 140 g, 10 g and $e = 1$.
4. Virtual goal: for each $\mathcal{B} \in \mathbb{B}_2$, launch the puck in the simulator from $p_c$ toward $\bar y_\mathcal{B}$. It must score after exactly the bounces in $\mathcal{B}$, in order, with a flight time matching $T_\mathcal{B}$.
5. The fine chain with pure reflection reproduces the coarse solution.
6. CasADi gradient check at random points, and convergence from the cold start in three scenarios: stationary puck, incoming puck, and a puck that requires going around it.

Closed loop:

7. Till's protocol: conversion over fixed seeds, delays of 0, 50 and 100 ms, opponents none, static and moving, with solve times logged.
