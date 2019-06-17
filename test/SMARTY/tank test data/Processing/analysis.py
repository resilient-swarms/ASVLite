import numpy as np
import pandas as pd
pd.options.mode.chained_assignment = None
pd.options.display.max_rows = 20


def read_waves(filename):
    'Read elevation data from the wave probe'
    df = pd.read_csv(filename,usecols=[0,1],names=["time","eta"])
    df['time'] = pd.to_datetime(df.time)
    df['time'] = (df.time-df.time.min()).dt.total_seconds()
    return df

def read_fixed_strain(name):
    return pd.read_csv(name,header=22,sep='\t',usecols=[0,1],names=['time','voltage'])

def read_qualisys(filename,off):
    'Read 6DOF data from the qualisys file'
    cNames = ['q_'+str(i+1) for i in range(6)]
    df = pd.read_csv(filename,sep='\t',header=11,usecols=[off+i for i in range(6)],names=cNames)
    df.loc[:,cNames[:3]] /= 1000
    df['time'] = np.arange(len(df.q_1))*0.01
    return df
def read_smarty(filename): return read_qualisys(filename,0)
def read_moored(filename): return read_qualisys(filename,17)

def read_moored_strain(filename):
    df = pd.read_csv(filename,names=['time','voltage'],header=0)
    df.time = df.time-df.time.iloc[0]
    return df

def interp_bad(df,bad):
    'overwrite bad values in a signal by linearly interpolating nearby points'
    ix = df.index[bad].tolist()  # bad row indices
    good = df[-bad]              # everything else is ok
    new = df.copy()
    for var in df:               # loop through columns
        if var=='time':continue    # other than time
        new.loc[ix,var] = np.interp(df.loc[ix,'time'],good.time,good[var])  # interpolate
    return new

def overwrite_extremes(df,var,f=3):
    'apply interp_bad to points far outside the median deviation'
    extreme = abs(df[var]-df[var].mean())>f*df[var].mad() # extreme rows
    return interp_bad(df,extreme)

def window_amp(df,var,f=2.1):
    'window based on the amplitude of var'
    sr = df[var].rolling(10,center=True).mean()-df[var].mean() # smoothed zero-mean var
    big = df[(sr>f*sr.mad()) & (sr>sr.shift(1)) & (sr>sr.shift(-1))].time # big peak times
    return df[(df.time>big.min()) & (df.time<big.max())] # window the data

def take_FFT(df,tname='time'):
    'Take the Fast Fourier Transform (FFT) of a data-frame.'
    hat_df = pd.DataFrame()                    # make empty data frame
    n = len(df)//2+1                           # number of points
    T = df[tname].max()-df[tname].min()        # signal period
    hat_df['freq'] = np.linspace(0,n/T,n)      # corresponding frequency range
    for col in df:                             # loop through the columns ...
        if(col == tname): continue                # ... other than time
        hat_df[col] = abs(np.fft.rfft(df[col])/n) # ... and take FFT
    return hat_df

def signal_stats(df,var,tname='time'):
    'Use FFT to get fundamental frequency and amplitude and mean'
    mean = df[var].mean()
    df_hat = take_FFT(df,tname).drop(0)
    ix = abs(df_hat[var]).idxmax()
    return pd.Series({'mean':mean,
            'amp':df_hat.loc[ix][var],
            'freq':df_hat.loc[ix]['freq']})

from scipy import signal
def low_pass(sr,dt=10,Hz=100):
    'Apply lowpass filter to a data series'
    win=signal.hann(Hz*dt+1)
    pad = np.zeros(len(win)//2) * np.NaN
    sig = signal.convolve(sr, win, mode='valid')/ sum(win)
    return pd.Series(np.r_[pad,sig,pad])

def srange(sr):
    'Find the range of a data series'
    return sr.max()-sr.min()

def resample_evenly(df,Hz=100):
    'Resample a DataFrame onto an evenly spaced time series'
    even_df = pd.DataFrame()
    even_df['time'] = np.arange(df.time.min(),df.time.max(),1/Hz)
    for name in df:
        if(name == 'time'): continue
        even_df[name] = np.interp(even_df.time,df.time,df[name])
    return even_df


import functools
def try_catch(func):
    'decorator to print exceptions from a function without stopping a loop'
    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except Exception as inst:
            print(*args, **kwargs)
            print(type(inst))     # the exception instance
            print(inst.args)      # arguments stored in .args
            return None
    return wrapper