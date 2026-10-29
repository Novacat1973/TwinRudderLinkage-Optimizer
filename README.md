# Twin-rudder linkage

A browser tool for designing the push-rod linkage of a **twin-rudder system steered by one tiller**. It computes the full 3D kinematics of both rudders, including rudder axes canted outward, and optimizes the tiller joint positions and rudder arm angle so that both blades move together, with the inner rudder turning only slightly further in a turn.

The whole app is a single `index.html`. There is no build step and no install.

## Why

Twin rudders driven from a central tiller through two rods rarely move symmetrically by accident. A small mismatch between the tiller joint position and the rudder arm angle produces very different deflections on port and starboard. In the worst case, the outer rudder stalls near a dead centre while the inner one keeps turning.

This tool shows that behaviour directly and finds dimensions that avoid it.

## Features

- **3D kinematics.** Each side is solved as a spatial four-bar linkage (tiller, rod, rudder arm, hull). Canted rudder axes and height offsets between the joints are handled exactly.
- **Live plan view.** Drag the tiller slider and watch both rudders, arms and rods move. Rudder angles are shown on the drawing.
- **Charts.**
  - Rudder angle against tiller angle, for the inner and outer rudder, with target curves.
  - Extra angle on the inner rudder.
- **Optimizer.** Each optimizable value has a **Fixed / Optimize** switch, so you decide what may change. A Nelder–Mead search fits both rudders to target curves for ratio and inner-rudder extra.
- **Transmission angle check.** The tool reports the smallest angle between rod and arm over the whole sweep. It warns if the linkage reaches dead centre.
- **Results table.**
  - ratio around centre
  - angles at full and half sweep
  - deviation from target
  - rod length
  - tiller joint radius and angle
- **Light and dark mode.** Inputs are stored in the browser’s `localStorage`.

## Usage

Open `index.html` in any current browser. To host it, enable GitHub Pages for the repository and serve the root folder.

Work through the page in three steps:

1. Enter the boat’s geometry and the rudder arm.
2. Set the targets: tiller sweep, ratio rudder : tiller, inner rudder extra and minimum transmission angle.
3. Set the values the optimizer may change to **Optimize** and press **Run optimizer**.

Values set to **Optimize** are only starting points and get overwritten. Values set to **Fixed** stay exactly as entered.

## Inputs and reference system

Origin is the **tiller pivot**. The axes are:

- x forward
- y to starboard
- z up

Both sides are mirror images.

| Input | Symbol | Meaning |
|---|---|---|
| A aft of tiller pivot | ΔX | Horizontal fore-and-aft distance from tiller pivot to axis point A |
| A off centreline | ΔY | Horizontal distance from the centreline to axis point A, each side |
| Rudder axis cant | c | Outward tilt of the rudder axis from vertical |
| Arm length | r | Distance from the rudder axis to the rudder joint centre |
| Arm angle | β | Angle of the arm in the arm plane, 0 = in line with the blade pointing forward, negative = outward |
| Ahead of tiller pivot | x | Tiller joint position along the tiller axis |
| Sideways, each joint | y | Tiller joint offset from the tiller axis, positive = same side as its rudder |
| Joint height offset | Δz | Tiller joint centre above rudder joint centre with the tiller centred, 0 = rods level |

**Axis point A.** Put a plane at right angles to the rudder axis through the centre of the rudder joint. This is the *arm plane*. A is the point where the rudder axis passes through it.

**Measuring from the upper fitting instead.** If the arm plane lies a distance *h* below the upper rudder fitting, measured along the axis, then A sits *h*·sin *c* further outboard. Add that to ΔY. The absolute heights of tiller pivot and rudder bearings do not matter; only Δz does.

**Joints.** Because the rudder joints move on a circle tilted by the cant angle, the rods move in 3D. Use spherical joints (rod ends or ball joints) at both ends of each rod.

## How it works

### Kinematics

For each side (sign *s* = −1 for port, +1 for starboard):

$$
A = (-\Delta X,\ s\,\Delta Y,\ 0),\qquad
\mathbf e_x = (1,0,0),\qquad
\mathbf e_{in} = (0,\ -s\cos c,\ -\sin c)
$$

$\mathbf e_{in}$ lies in the arm plane and points inboard. The rudder joint at arm angle θ is

$$
B(\theta) = A + r\,(\cos\theta\,\mathbf e_x + \sin\theta\,\mathbf e_{in}),\qquad \theta_0 = \beta
$$

The tiller joint rotates about the vertical tiller axis:

$$
T(\psi) = R_z(\psi)\,T_0,\qquad T_0 = (x,\ s\,y,\ B_z(\theta_0) + \Delta z)
$$

The rod length is fixed so that both rudders sit at 0° with the tiller centred:

$$
L = \lvert B(\theta_0) - T_0 \rvert
$$

For a given tiller angle ψ, set $\mathbf d = A - T(\psi)$. The rod-length condition becomes

$$
p\cos\theta + q\sin\theta = k,\qquad
p = \mathbf d\cdot\mathbf e_x,\quad q = \mathbf d\cdot\mathbf e_{in},\quad
k = \frac{L^2 - \lvert\mathbf d\rvert^2 - r^2}{2r}
$$

It has the closed-form solution

$$
\theta = \operatorname{atan2}(q,p) \pm \arccos\!\frac{k}{\sqrt{p^2+q^2}}
$$

The branch closest to the neutral position is used. If $\lvert k\rvert > \sqrt{p^2+q^2}$, there is no solution and the linkage has jammed.

The rudder angle is the rotation about the rudder’s own canted axis, δ = θ − θ₀, with the sign set so that positive means trailing edge to port. With the boat upright, this equals the blade’s angle of attack.

### Transmission angle

At each rudder, μ is the angle between the rod and the arm. At the tiller, it is the angle between the rod and the tiller crank. Values are folded into 0–90°, where 90° is ideal. The rod force rises with 1/sin μ. Below about 40° the linkage becomes stiff and sensitive to play.

### Optimizer

Target curves for tiller angles 0 ≤ ψ ≤ ψ<sub>max</sub>, with ratio *i* and inner-rudder extra Δ:

$$
\delta^{*}_{inner} = i\,\psi + \tfrac12\,\Delta\,(\psi/\psi_{max})^2,\qquad
\delta^{*}_{outer} = i\,\psi - \tfrac12\,\Delta\,(\psi/\psi_{max})^2
$$

Cost function, evaluated at 24 tiller angles:

$$
J = \frac1N\sum\Big[(\delta_{inner}-\delta^{*}_{inner})^2 + (\delta_{outer}-\delta^{*}_{outer})^2\Big]
\;+\; 0.05\,\max(0,\ \mu_{lim}-\mu_{min})^2
$$

When three values are set to Optimize, the problem is under-determined, because there are only two targets. A small extra term then favours the solution with the largest transmission angle.

The search uses a Nelder–Mead simplex with two starting points:

- the current inputs
- a parallelogram linkage (x = i·r, y = 0, β = 0)

Each start is followed by two restarts with smaller steps. A configuration that jams anywhere in the sweep is rejected.

## Design guidance

| Choice | Typical value | Notes |
|---|---|---|
| Maximum rudder angle | 30–35° | Beyond that a rudder mostly brakes. Set mechanical stops just past the design sweep. |
| Ratio rudder : tiller | 0.9–1.0 | Tiller force grows linearly with the ratio. Smaller ratios need more tiller travel. |
| Inner rudder extra | 0.5–2° at full sweep | Estimate: Δ ≈ (2b/l)·sin²β, with half the rudder spacing b, distance l from the hull’s turning point to the rudders, and inflow angle β at the stern. Full car-style Ackermann is too much. |
| Arm angle | β ≈ −atan(ΔX / (ΔY·cos c)) | Puts the rod at right angles to the arm with the tiller centred, which gives the best transmission angle. |
| Minimum transmission angle | ≥ 40° | Check it by moving the slider beyond the design sweep. |

Optimize **exactly two values** for the two targets:

- **Joint spacing on the tiller is fixed by the design:** optimize *tiller joint ahead* and *arm angle*.
- **Rudder arm must stay as it is:** optimize *tiller joint ahead* and *sideways*.

The arm angle mainly shifts the inner-rudder extra. The tiller joint distance mainly sets the ratio.

## Assumptions and limitations

- The tiller pivot axis is vertical.
- The rudder axes have no rake. They are canted sideways only.
- Port and starboard are mirror images.
- All parts are rigid. Play and elasticity are not modelled.
- The kinematics are for the upright boat. Heel changes the inflow, not the linkage.

This is a geometry tool. Check loads, joint sizes and clearances separately.

## Project structure

```
index.html          the complete app (HTML, CSS, JavaScript)
README.md
docs/screenshot.png
```

The only external resource is the Barlow typeface from Google Fonts. Without a network connection, the page falls back to system fonts and works the same.

## Contributing

Issues and pull requests are welcome. Rake of the rudder axes, asymmetric layouts and export of joint coordinates would be useful extensions.

## License

_Add a license of your choice, for example MIT._
