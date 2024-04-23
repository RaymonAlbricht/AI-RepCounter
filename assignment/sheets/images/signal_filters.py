import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, lfilter

# Parameters
n = 500  # number of samples
t = np.linspace(0, 4 * np.pi, n)  # time vector

# Deterministic signal (Sine wave)
deterministic_signal = np.sin(t) + .5

# Stochastic signal (Pure Gaussian noise)
stochastic_signal = np.random.normal(0, 0.5, n) + np.sin(t) + .5

# Function to calculate basic metrics
def calculate_metrics(signal):
    metrics = {
        'variance': np.var(signal),
        'peak_to_peak': np.ptp(signal),
        'mean': np.mean(signal),
        'min': np.min(signal),
        'max': np.max(signal),
        'energy': np.sum(signal**2)
    }
    return metrics

# Root Mean Square (RMS) calculation
def rms(signal):
    return np.sqrt(np.mean(signal**2))

# Crest Factor calculation
def crest_factor(signal):
    peak = np.max(np.abs(signal))
    rms_val = rms(signal)
    return peak / rms_val if rms_val != 0 else 0

# Skewness calculation
def skewness(signal):
    return (np.mean((signal - np.mean(signal))**3)) / (np.std(signal)**3)

# Zero Crossing Rate calculation
def zero_crossing_rate(signal):
    return ((signal[:-1] * signal[1:]) < 0).sum() / len(signal)

# Calculate metrics for both signals
deterministic_metrics = calculate_metrics(deterministic_signal)
stochastic_metrics = calculate_metrics(stochastic_signal)

deterministic_rms = rms(deterministic_signal)
deterministic_cf = crest_factor(deterministic_signal)
deterministic_skewness = skewness(deterministic_signal)
deterministic_zcr = zero_crossing_rate(deterministic_signal)

stochastic_rms = rms(stochastic_signal)
stochastic_cf = crest_factor(stochastic_signal)
stochastic_skewness = skewness(stochastic_signal)
stochastic_zcr = zero_crossing_rate(stochastic_signal)


# Butterworth Filter Design
def butter_filter(order, cutoff, fs, btype):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype=btype, analog=False)
    return b, a

# Highpass Filter
def highpass_filter(signal, cutoff, fs, order=3):
    b, a = butter_filter(order, cutoff, fs, 'high')
    return lfilter(b, a, signal)

# Lowpass Filter
def lowpass_filter(signal, cutoff, fs, order=3):
    b, a = butter_filter(order, cutoff, fs, 'low')
    return lfilter(b, a, signal)

# Filter application
fs = n / (4 * np.pi)  # Sampling frequency
cutoff_highpass = 0.25  # Highpass filter cutoff frequency
cutoff_lowpass = 0.25   # Lowpass filter cutoff frequency

highpass_deterministic = highpass_filter(deterministic_signal, cutoff_highpass, fs)
lowpass_stochastic = lowpass_filter(stochastic_signal, cutoff_lowpass, fs)

# Calculate metrics for both signals
deterministic_metrics = calculate_metrics(highpass_deterministic)
stochastic_metrics = calculate_metrics(lowpass_stochastic)

deterministic_rms = rms(deterministic_signal)
deterministic_cf = crest_factor(deterministic_signal)
deterministic_skewness = skewness(deterministic_signal)
deterministic_zcr = zero_crossing_rate(deterministic_signal)

stochastic_rms = rms(stochastic_signal)
stochastic_cf = crest_factor(stochastic_signal)
stochastic_skewness = skewness(stochastic_signal)
stochastic_zcr = zero_crossing_rate(stochastic_signal)

# Plotting
fig, axs = plt.subplots(2, 1, figsize=(10, 6))

# Plotting Deterministic Signal with Highpass Filtered Signal
axs[0].plot(t, deterministic_signal, label='Original Deterministic Signal')
axs[0].axhline(deterministic_metrics['mean'], color='r', linestyle='--', label='Mean')
axs[0].axhline(deterministic_metrics['max'], color='g', linestyle=':', label='Max')
axs[0].axhline(deterministic_metrics['min'], color='b', linestyle=':', label='Min')
axs[0].plot(t, highpass_deterministic, label='Highpass Filtered')
axs[0].set_title('Deterministic Signal and Highpass Filtered Signal')
# Displaying all metrics
axs[0].annotate(f'RMS: {deterministic_rms:.2f}\nCF: {deterministic_cf:.2f}\nSkewness: {deterministic_skewness:.2f}\nZCR: {deterministic_zcr:.2f}\nVariance: {deterministic_metrics["variance"]:.2f}',
                xy=(0.8, 0.6), xycoords='axes fraction')
##axs[0].legend()

# Plotting Stochastic Signal with Lowpass Filtered Signal
axs[1].plot(t, stochastic_signal, label='Original Stochastic Signal')
axs[1].plot(t, lowpass_stochastic, label='Lowpass Filtered')
axs[1].axhline(stochastic_metrics['mean'], color='r', linestyle='--', label='Mean')
axs[1].axhline(stochastic_metrics['max'], color='g', linestyle=':', label='Max')
axs[1].axhline(stochastic_metrics['min'], color='b', linestyle=':', label='Min')
axs[1].set_title('Stochastic Signal and Lowpass Filtered Signal')
# Displaying all metrics
axs[1].annotate(f'RMS: {stochastic_rms:.2f}\nCF: {stochastic_cf:.2f}\nSkewness: {stochastic_skewness:.2f}\nZCR: {stochastic_zcr:.2f}\nVariance: {stochastic_metrics["variance"]:.2f}',
                xy=(0.8, 0.6), xycoords='axes fraction')
##axs[1].legend()

for ax in axs:
    ax.grid()

plt.tight_layout()
plt.show()
##
### Plotting
##fig, axs = plt.subplots(4, 1, figsize=(12, 12))
##
### Original Deterministic Signal
##axs[0].plot(t, deterministic_signal, label='Original Deterministic Signal')
##axs[0].set_title('Original Deterministic Signal')
##
### Highpass Filtered Deterministic Signal
##axs[1].plot(t, highpass_deterministic, label='Highpass Filtered')
##axs[1].set_title('Highpass Filtered Deterministic Signal')
##
### Original Stochastic Signal
##axs[2].plot(t, stochastic_signal, label='Original Stochastic Signal')
##axs[2].set_title('Original Stochastic Signal')
##
### Lowpass Filtered Stochastic Signal
##axs[3].plot(t, lowpass_stochastic, label='Lowpass Filtered')
##axs[3].set_title('Lowpass Filtered Stochastic Signal')
##
##for ax in axs:
##    ax.legend()
##    ax.grid()
##
##plt.tight_layout()
##plt.show()
##
##
##
####
##### Plotting
####fig, axs = plt.subplots(2, 1, figsize=(12, 8))
####
##### Plotting Deterministic Signal
####axs[0].plot(t, deterministic_signal, label='Deterministic Signal')
####axs[0].axhline(deterministic_metrics['mean'], color='r', linestyle='--', label='Mean')
####axs[0].axhline(deterministic_metrics['max'], color='g', linestyle=':', label='Max')
####axs[0].axhline(deterministic_metrics['min'], color='b', linestyle=':', label='Min')
##### Displaying all metrics
####axs[0].annotate(f'RMS: {deterministic_rms:.2f}\nCF: {deterministic_cf:.2f}\nSkewness: {deterministic_skewness:.2f}\nZCR: {deterministic_zcr:.2f}\nVariance: {deterministic_metrics["variance"]:.2f}',
####                xy=(0.7, 0.7), xycoords='axes fraction')
####axs[0].set_title('Deterministic Signal with Metrics')
####axs[0].legend()
####
##### Plotting Stochastic Signal
####axs[1].plot(t, stochastic_signal, label='Stochastic Signal')
####axs[1].axhline(stochastic_metrics['mean'], color='r', linestyle='--', label='Mean')
####axs[1].axhline(stochastic_metrics['max'], color='g', linestyle=':', label='Max')
####axs[1].axhline(stochastic_metrics['min'], color='b', linestyle=':', label='Min')
##### Displaying all metrics
####axs[1].annotate(f'RMS: {stochastic_rms:.2f}\nCF: {stochastic_cf:.2f}\nSkewness: {stochastic_skewness:.2f}\nZCR: {stochastic_zcr:.2f}\nVariance: {stochastic_metrics["variance"]:.2f}',
####                xy=(0.7, 0.7), xycoords='axes fraction')
####axs[1].set_title('Stochastic Signal (Gaussian Noise) with Metrics')
####axs[1].legend()
####
####plt.tight_layout()
####plt.show()
