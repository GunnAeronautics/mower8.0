import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

gravity = 9.80665
targetTime = 37000
deltaT = 20  # ms

def compute_and_plot(ax, alt_initial, noParachuteTermVelo):
    ax.clear()
    for n in range(2, 16):
        codeVelo = 5 + n/10
        deltas = []
        realTermVelos = []
        for j in range(2, 21):
            realTermVelo = 5 + j/10
            time = 10000
            alt = alt_initial
            parachuteDeployed = False

            altDeployed = codeVelo*(alt - (targetTime - time)/1000.0 * noParachuteTermVelo)\
                          /(codeVelo - noParachuteTermVelo)
            if altDeployed < 5:
                altDeployed = 5
            time += (alt - altDeployed)/noParachuteTermVelo*1000.0
            time += altDeployed / realTermVelo*1000.0

            # if round(abs(37.5 - (time/1000.0)), 3) < 1.5:
            if True:
                deltas.append(round(abs(37.5 - (time/1000.0)), 3))
                realTermVelos.append(realTermVelo)

        ax.plot(realTermVelos, deltas,
                label="Predicted Velocity " + str(codeVelo),
                color=(n/20, n/40, n/40))

    ax.axhline(y=1.5, color='blue', linestyle='--', linewidth=2, label="0 point cutoff")
    ax.set_xlabel("Real Terminal Velocity (m/s)")
    ax.set_ylabel("Delta Time (s)")
    ax.set_title(f"Time Difference vs Real Term Velo  "
                 f"(alt(10 sec) = {alt_initial:.1f} m, noChuteV = {noParachuteTermVelo:.2f} m/s)")
    ax.grid(True)
    ax.legend(loc="upper center", fontsize=8)
    ax.set_xlim(5.0, 7.0)

# Set up figure with room for two sliders
fig, ax = plt.subplots(figsize=(10, 7))
plt.subplots_adjust(bottom=0.25)

initial_alt = 250
initial_no_chute = 9.0
compute_and_plot(ax, initial_alt, initial_no_chute)

# Slider axes: [left, bottom, width, height]
ax_slider_alt = plt.axes([0.15, 0.10, 0.7, 0.03])
alt_slider = Slider(
    ax=ax_slider_alt,
    label='Altitude 10 seconds in (m)',
    valmin=190,
    valmax=215,
    valinit=initial_alt,
    valstep=1,
)

ax_slider_nochute = plt.axes([0.15, 0.04, 0.7, 0.03])
nochute_slider = Slider(
    ax=ax_slider_nochute,
    label='Reefed Parachute Terminal Velocity (m/s)',
    valmin=7,
    valmax=15.0,
    valinit=initial_no_chute,
    valstep=0.1,
)

def update(val):
    compute_and_plot(ax, alt_slider.val, nochute_slider.val)
    fig.canvas.draw_idle()

alt_slider.on_changed(update)
nochute_slider.on_changed(update)

plt.show()