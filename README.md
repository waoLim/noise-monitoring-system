# Development of a Fixed IoT-Based Outdoor Noise Monitoring System

Abstract—Noise is the unwanted sound emitted from daily
human activities that can cause harmful health effects from
prolonged exposure. Existing noise monitoring solutions are
limited by the type of data they collect and the number of edge
devices being deployed. This restricts the potential applications
that could be developed in conjunction with the collected noise
data. The developed monitoring system measured the continuous
equivalent A-weighted sound pressure level (LAeq) for noise
level quantification, and the Mel-frequency Cepstrum Coefficients
(MFCC) were extracted to enhance the system’s analyzing capa-
bilities for further in-depth studies such as noise classification.
Four sensor nodes were deployed, each comprised of an INMP441
microphone, a Teensy 4.1 for edge computing, and an ESP32 and
LPSTK-CC1352R for wireless communication. Wi-Fi, Bluetooth
Low-Energy (BLE), and ZigBee protocols were explored to
determine which is the most suitable network to use for the
system. Overall, the system exhibited reliable LAeq measurements
with an average error of 0.22 dBA when compared to a PAA3x
audio analyzer and an R² value of 0.9977, corresponding to
a strong linear response. The MFCC computation achieved
acceptable accuracy with efficient runtime despite hardware
constraints. The system was successfully deployed using Wi-Fi in
two outdoor locations with varying noise levels and Wi-Fi signal
strengths, with payload transmission success rates ranging from
90% to 94%.
