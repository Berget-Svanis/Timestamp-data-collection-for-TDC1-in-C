# Timestamp-data-collection-for-TDC1-in-C
Data collection in C from TDC1 time to digital converter for use in lab work and bachelor thesis

This repository contains two programs: timestamp_calculation.c is used for acquring data and calculating coincidence histogram from a TDC1 box and plot_timestamp.py is used for plotting the results calculated by the C program. 

These programs were created in conjunction with a bachelor thesis at KTH and were used together with a lab setup to demonstrate quantum entanglement and the Hong-Ou-Mandel effect. This program (or iterations of it) will in the future be used in a lab setup for courses and thus these programs exist here for further usage and availability.

For more information on the TDC1 time to digial converter please check out their repository: https://github.com/s-fifteen-instruments/Timestamp_TDC1

Please note that this program implements multi-threading and is programmed for a Raspberry Pi 4. In order to compile the C program correctly the following command should be used: 

```
cc -fPIC -shared -fopenmp -o calc.so timestamp_calculation.c
```

For further questions or comments: esvanb@kth.se
