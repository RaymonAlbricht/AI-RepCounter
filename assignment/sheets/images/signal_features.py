import numpy as np
import matplotlib.pyplot as plt

# Parameters
n = 500  # number of samples
t = np.linspace(0, 4 * np.pi, n)  # time vector

# Deterministic signal (Sine wave)
deterministic_signal = np.sin(t)

# Stochastic signal (Pure Gaussian noise)
stochastic_signal = np.random.normal(0, 0.5, n)

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

# Plotting
fig, axs = plt.subplots(2, 1, figsize=(10, 6))

# Plotting Deterministic Signal
axs[0].plot(t, deterministic_signal, label='Deterministic Signal')
axs[0].axhline(deterministic_metrics['mean'], color='r', linestyle='--', label='Mean')
axs[0].axhline(deterministic_metrics['max'], color='g', linestyle=':', label='Max')
axs[0].axhline(deterministic_metrics['min'], color='b', linestyle=':', label='Min')
# Displaying all metrics
axs[0].annotate(f'RMS: {deterministic_rms:.2f}\nCF: {deterministic_cf:.2f}\nSkewness: {deterministic_skewness:.2f}\nZCR: {deterministic_zcr:.2f}\nVariance: {deterministic_metrics["variance"]:.2f}',
                xy=(0.8, 0.6), xycoords='axes fraction')
axs[0].set_title('Deterministic Signal with Metrics')
##axs[0].legend()

# Plotting Stochastic Signal
axs[1].plot(t, stochastic_signal, label='Stochastic Signal')
axs[1].axhline(stochastic_metrics['mean'], color='r', linestyle='--', label='Mean')
axs[1].axhline(stochastic_metrics['max'], color='g', linestyle=':', label='Max')
axs[1].axhline(stochastic_metrics['min'], color='b', linestyle=':', label='Min')
# Displaying all metrics
axs[1].annotate(f'RMS: {stochastic_rms:.2f}\nCF: {stochastic_cf:.2f}\nSkewness: {stochastic_skewness:.2f}\nZCR: {stochastic_zcr:.2f}\nVariance: {stochastic_metrics["variance"]:.2f}',
                xy=(0.8, 0.6), xycoords='axes fraction')
axs[1].set_title('Stochastic Signal (Gaussian Noise) with Metrics')
##axs[1].legend()

plt.tight_layout()
plt.show()
