

import matplotlib.pyplot as plt
import numpy as np
import os


def get_data(pause=True, pause_start=0, pause_seconds=0.2):

    fname = "foopause.data" if pause else "foo.data"
    p = "-DPAUSE_EXPERIMENT" if pause else ""
    ps = "-DPAUSE_SECONDS=" + str(pause_seconds) if pause else ""
    pt = "-DPAUSE_START_TIME=" + str(pause_start) if pause else ""

    cargs = "CROSS_COMPILE= ARM_COMPILE_FLAGS='{pause}"\
            " {psec} {pstart}' make -j4"

    print(cargs.format(pause=p, psec=ps, pstart=pt))
    os.system(cargs.format(pause=p, psec=ps, pstart=pt))
    os.system('./machine-control -N  --nohoming-required '
              '--norange-check -c sample.config test.ngc > ' + '/tmp/' + fname)

    data = np.genfromtxt('/tmp/' + fname, delimiter=None, skip_header=1,
                         skip_footer=0,
                         names=['t', 'timerloop', 'euclidspeed', 'euclidaccel',
                                'x', 'vx', 'ax', 'y', 'vy', 'ay', 'z', 'vz',
                                'az'])

    return data

plt.close('all')
data = get_data(pause=True, pause_start=2)
data2 = get_data(pause=False)
f, axarr = plt.subplots(2, sharex=True)
axarr[0].plot(data['t'], data['euclidspeed'], color='r')
axarr[0].plot(data2['t'], data2['euclidspeed'], color='b')
axarr[0].set_title('Speed and accel')
axarr[1].plot(data['t'], data['euclidaccel'])
axarr[0].set_xlim([-1, 10])

plt.show()
