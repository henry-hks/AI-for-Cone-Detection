import numpy as np
from scipy.signal import butter, filtfilt

def butter_lowpass_filter(data, cutoff, fs, order, nyq):
    normal_cutoff = cutoff / nyq
    # get the filter coefficients
    b, a = butter(order, normal_cutoff, btype = "low", analog = False)
    y = filtfilt(b, a, data, axis=0)
    
    return y

