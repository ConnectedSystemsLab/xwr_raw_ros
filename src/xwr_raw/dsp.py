#!/usr/bin/env python3

"""Helper functions for signal processing.
"""

import numpy as np
import cv2
from numba import njit, objmode

""" Functions to convert from RadarFrame messages to radar cubes.. """

def reshape_frame(frame, flip_ods_phase=False, flip_aop_phase=False):
    """ Use this to reshape RadarFrameFull messages.

    Args:
        frame (RadarFrameFull): RadarFrameFull message.
        flip_ods_phase (bool): Flip the phase of RX2 and RX3.
        flip_aop_phase (bool): Flip the phase of RX1 and RX3.

    Returns:
        radar_cube (np.ndarray): Radar cube. 
    """

    platform = frame.platform
    adc_output_fmt = frame.adc_output_fmt
    rx_phase_bias = np.array([a + 1j*b for a,b in zip(frame.rx_phase_bias[0::2],
                                                      frame.rx_phase_bias[1::2])])

    n_chirps  = int(frame.shape[0])
    rx        = np.array([int(x) for x in frame.rx])
    n_rx      = int(frame.shape[1])
    tx        = np.array([int(x) for x in frame.tx])
    n_tx      = int(sum(frame.tx))
    n_samples = int(frame.shape[2])

    return _reshape_frame(np.array(frame.data),
                          platform, adc_output_fmt, rx_phase_bias,
                          n_chirps, rx, n_rx, tx, n_tx, n_samples,
                          flip_ods_phase=flip_ods_phase,
                          flip_aop_phase=flip_aop_phase)

@njit(cache=True)
def _reshape_frame(data,
                   platform, adc_output_fmt, rx_phase_bias,
                   n_chirps, rx, n_rx, tx, n_tx, n_samples,
                   flip_ods_phase=False,
                   flip_aop_phase=False):
    if adc_output_fmt > 0:

        radar_cube = np.zeros(len(data) // 2, dtype=np.complex64)

        radar_cube[0::2] = 1j*data[0::4] + data[2::4]
        radar_cube[1::2] = 1j*data[1::4] + data[3::4]

        radar_cube = radar_cube.reshape((n_chirps, 
                                         n_rx, 
                                         n_samples))

        # Apply RX phase correction for each antenna. 
        if 'xWR68xx' in platform:
            if flip_ods_phase: # Apply 180 deg phase change on RX2 and RX3
                c = 0
                for i_rx, rx_on in enumerate(rx):
                    if rx_on:
                        if i_rx == 1 or i_rx == 2:
                            radar_cube[:,c,:] *= -1
                        c += 1
            elif flip_aop_phase: # Apply 180 deg phase change on RX1 and RX3
                c = 0
                for i_rx, rx_on in enumerate(rx):
                    if rx_on:
                        if i_rx == 0 or i_rx == 2:
                            radar_cube[:,c,:] *= -1
                        c += 1


        radar_cube = radar_cube.reshape((n_chirps//n_tx, 
                                         n_rx*n_tx, 
                                         n_samples))

        # Apply RX phase correction from calibration.
        c = 0
        for i_tx, tx_on in enumerate(tx):
            if tx_on:
                for i_rx, rx_on in enumerate(rx):
                    if rx_on:
                        v_rx = i_tx*len(rx) + i_rx
                        # print(v_rx)
                        radar_cube[:,c,:] *= rx_phase_bias[v_rx]
                        c += 1

    else:
        radar_cube = data.reshape((n_chirps//n_tx, 
                                   n_rx*n_tx, 
                                   n_samples)).astype(np.complex64)

    return radar_cube

def reshape_frame_tdm(frame, flip_ods_phase=False, flip_aop_phase=False):
    """ Use this to reshape RadarFrameFull messages if using TDM.

    Args:
        frame (RadarFrameFull): RadarFrameFull message.
        flip_ods_phase (bool): Flip the phase of RX2 and RX3.
        flip_aop_phase (bool): Flip the phase of RX1 and RX3.

    Returns:
        radar_cube (np.ndarray): Radar cube.
    """

    platform = frame.platform
    adc_output_fmt = frame.adc_output_fmt
    rx_phase_bias = np.array([a + 1j*b for a,b in zip(frame.rx_phase_bias[0::2],
                                                      frame.rx_phase_bias[1::2])])

    n_chirps  = int(frame.shape[0])
    rx        = np.array([int(x) for x in frame.rx])
    n_rx      = int(frame.shape[1])
    tx        = np.array([int(x) for x in frame.tx])
    n_tx      = int(sum(frame.tx))
    n_samples = int(frame.shape[2])

    return _reshape_frame_tdm(np.array(frame.data),
                              platform, adc_output_fmt, rx_phase_bias,
                              n_chirps, rx, n_rx, tx, n_tx, n_samples,
                              flip_ods_phase=flip_ods_phase,
                              flip_aop_phase=flip_aop_phase)

@njit(cache=True)
def _tdm(radar_cube, n_tx, n_rx):
    """ Implements "Compensation of Motion-Induced Phase Errors in TDM MIMO Radars
        https://d-nb.info/1161008624/34
    """
    radar_cube_tdm = np.zeros((radar_cube.shape[0]*n_tx, 
                               radar_cube.shape[1], 
                               radar_cube.shape[2]), 
                               dtype=np.complex64)

    for i in range(n_tx):
        radar_cube_tdm[i::n_tx,i*n_rx:(i+1)*n_rx] \
                = radar_cube[:,i*n_rx:(i+1)*n_rx]

    return radar_cube_tdm

@njit(cache=True)
def _reshape_frame_tdm(data,
                       platform, adc_output_fmt, rx_phase_bias,
                       n_chirps, rx, n_rx, tx, n_tx, n_samples,
                       flip_ods_phase=False,
                       flip_aop_phase=False):


    radar_cube = _reshape_frame(data, 
                                platform, adc_output_fmt, rx_phase_bias,
                                n_chirps, rx, n_rx, tx, n_tx, n_samples,
                                flip_ods_phase=flip_ods_phase,
                                flip_aop_phase=flip_aop_phase)

    radar_cube_tdm = _tdm(radar_cube, n_tx, n_rx)

    return radar_cube_tdm


""" AoA estimation functions. """

@njit(cache=True)
def get_mean(x, axis=0):
    return np.sum(x, axis=axis)/x.shape[axis]

@njit(cache=True)
def cov_matrix(x):
    """ Calculates the spatial covariance matrix (Rxx) for a given set of input data (x=inputData).
        Assumes rows denote Vrx axis.

    Args:
        x (ndarray): A 2D-Array with shape (rx, adc_samples) slice of the output of the 1D range fft

    Returns:
        Rxx (ndarray): A 2D-Array with shape (rx, rx)
    """

    #if x.ndim > 2:
    #    raise ValueError("x has more than 2 dimensions.")

    #if x.shape[0] > x.shape[1]:
    #    warnings.warn("cov_matrix input should have Vrx as rows. Needs to be transposed", RuntimeWarning)
    #    x = x.T

    _, num_adc_samples = x.shape
    x_T = x.T
    Rxx = x @ np.conjugate(x_T)
    Rxx = np.divide(Rxx, num_adc_samples)

    return Rxx 

@njit(cache=True)
def gen_steering_vec(ang_est_range, ang_est_resolution, num_ant):
    """Generate a steering vector for AOA estimation given the theta range, theta resolution, and number of antennas

    Defines a method for generating steering vector data input --Python optimized Matrix format
    The generated steering vector will span from -angEstRange to angEstRange with increments of ang_est_resolution
    The generated steering vector should be used for all further AOA estimations (bartlett/capon)

    Args:
        ang_est_range (int): The desired span of thetas for the angle spectrum.
        ang_est_resolution (float): The desired resolution in terms of theta
        num_ant (int): The number of Vrx antenna signals captured in the RDC

    Returns:
        num_vec (int): Number of vectors generated (integer divide angEstRange/ang_est_resolution)
        steering_vectors (ndarray): The generated 2D-array steering vector of size (num_vec,num_ant)

    Example:
        >>> #This will generate a numpy array containing the steering vector with 
        >>> #angular span from -90 to 90 in increments of 1 degree for a 4 Vrx platform
        >>> _, steering_vec = gen_steering_vec(90,1,4)

    """
    num_vec = (2 * ang_est_range / ang_est_resolution + 1)
    num_vec = int(round(num_vec))
    steering_vectors = np.zeros((num_vec, num_ant), dtype='complex64')
    for kk in range(num_vec):
        for jj in range(num_ant):
            mag = -1 * np.pi * jj * np.sin((-ang_est_range + kk * ang_est_resolution) * np.pi / 180)
            real = np.cos(mag)
            imag = np.sin(mag)

            steering_vectors[kk, jj] = np.complex(real, imag)

    return (num_vec, steering_vectors) 

@njit(cache=True)
def aoa_bartlett(steering_vec, sig_in, axis):
    """Perform AOA estimation using Bartlett Beamforming on a given input signal (sig_in). Make sure to specify the correct axis in (axis)
    to ensure correct matrix multiplication. The power spectrum is calculated using the following equation:

    .. math::
        P_{ca} (\\theta) = a^{H}(\\theta) R_{xx}^{-1} a(\\theta)

    This steers the beam using the steering vector as weights:

    .. math::
        w_{ca} (\\theta) = a(\\theta)

    Args:
        steering_vec (ndarray): A 2D-array of size (numTheta, num_ant) generated from gen_steering_vec
        sig_in (ndarray): Either a 2D-array or 3D-array of size (num_ant, numChirps) or (numChirps, num_vrx, num_adc_samples) respectively, containing ADC sample data sliced as described
        axis (int): Specifies the axis where the Vrx data in contained.

    Returns:
        doa_spectrum (ndarray): A 3D-array of size (numChirps, numThetas, numSamples)

    Example:
        >>> # In this example, dataIn is the input data organized as numFrames by RDC
        >>> frame = 0
        >>> dataIn = np.random.rand((num_frames, num_chirps, num_vrx, num_adc_samples))
        >>> aoa_bartlett(steering_vec,dataIn[frame],axis=1)
    """
    n_theta = steering_vec.shape[0]
    n_rx = sig_in.shape[1]
    n_range = sig_in.shape[2]
    y = np.zeros((sig_in.shape[0], n_theta, n_range), dtype='complex64')
    for i in range(sig_in.shape[0]):
        y[i] = np.conjugate(steering_vec) @ sig_in[i]
    # y = np.conjugate(steering_vec) @ sig_in
    return y 

@njit(cache=True)
def aoa_capon(x, steering_vector):
    """Perform AOA estimation using Capon (MVDR) Beamforming on a rx by chirp slice

    Calculate the aoa spectrum via capon beamforming method using one full frame as input.
    This should be performed for each range bin to achieve AOA estimation for a full frame
    This function will calculate both the angle spectrum and corresponding Capon weights using
    the equations prescribed below.

    .. math::
        P_{ca} (\\theta) = \\frac{1}{a^{H}(\\theta) R_{xx}^{-1} a(\\theta)}

        w_{ca} (\\theta) = \\frac{R_{xx}^{-1} a(\\theta)}{a^{H}(\\theta) R_{xx}^{-1} a(\\theta)}

    Args:
        x (ndarray): Output of the 1d range fft with shape (num_ant, numChirps)
        steering_vector (ndarray): A 2D-array of size (numTheta, num_ant) generated from gen_steering_vec
        magnitude (bool): Azimuth theta bins should return complex data (False) or magnitude data (True). Default=False

    Raises:
        ValueError: steering_vector and or x are not the correct shape

    Returns:
        A list containing numVec and steeringVectors
        den (ndarray: A 1D-Array of size (numTheta) containing azimuth angle estimations for the given range
        weights (ndarray): A 1D-Array of size (num_ant) containing the Capon weights for the given input data

    Example:
        >>> # In this example, dataIn is the input data organized as numFrames by RDC
        >>> Frame = 0
        >>> dataIn = np.random.rand((num_frames, num_chirps, num_vrx, num_adc_samples))
        >>> for i in range(256):
        >>>     scan_aoa_capon[i,:], _ = dss.aoa_capon(dataIn[Frame,:,:,i].T, steering_vector, magnitude=True)

    """


    Rxx = cov_matrix(x)
    Rxx_inv = np.linalg.inv(Rxx).astype(np.complex64)
    # Calculate Covariance Matrix Rxx
    first = Rxx_inv @ steering_vector.T
    #den = np.reciprocal(np.einsum('ij,ij->i', steering_vector.conj(), first.T))
    den = np.zeros(first.shape[1], dtype=np.complex64)
    steering_vector_conj = steering_vector.conj()
    first_T = first.T
    for i in range(first_T.shape[0]):
        for j in range(first_T.shape[1]):
            den[i] += steering_vector_conj[i,j] * first_T[i,j]
    den = np.reciprocal(den)

    weights = first @ den

    return den, weights


""" Radar cube processing functions. """

@njit(cache=True)
def compute_range_azimuth_capon(radar_cube,
                                angle_res=1,
                                angle_range=90):
    """ Compute the range azimuth Capon spectrum for a given radar cube.
    """

    n_range_bins = radar_cube.shape[2]
    n_rx = radar_cube.shape[1]
    n_chirps = radar_cube.shape[0]
    n_angle_bins = (angle_range * 2) // angle_res + 1

    range_cube = np.zeros_like(radar_cube)
    with objmode(range_cube='complex128[:,:,:]'):
        range_cube = np.fft.fft(radar_cube, axis=2)
    range_cube = np.transpose(range_cube, (2, 1, 0))
    range_cube = np.asarray(range_cube, dtype=np.complex64)

    _ , steering_vec = gen_steering_vec(angle_range, angle_res, n_rx)

    range_azimuth = np.zeros((n_range_bins, n_angle_bins), dtype=np.complex_)

    for r_idx in range(n_range_bins):
        range_azimuth[r_idx,:], _ = aoa_capon(range_cube[r_idx],
                                                  steering_vec)

    range_azimuth = np.log(np.abs(range_azimuth))

    return range_azimuth

@njit(cache=True)
def compute_range_azimuth_capon_real(radar_cube,
                                     angle_res=1,
                                     angle_range=90):
    """ Compute the range azimuth Capon spectrum for a given radar cube.
        Modified version for AWR2944.
    """

    n_range_bins = radar_cube.shape[2]//2
    n_rx = radar_cube.shape[1]
    n_chirps = radar_cube.shape[0]
    n_angle_bins = (angle_range * 2) // angle_res + 1

    range_cube = np.fft.fft(radar_cube, axis=2)[:,:,:radar_cube.shape[2]]
    range_cube = np.transpose(range_cube, (2, 1, 0))

    _ , steering_vec = gen_steering_vec(angle_range, angle_res, n_rx)

    range_azimuth = np.zeros((n_range_bins, n_angle_bins), dtype=np.complex_)

    for r_idx in range(n_range_bins):
        range_azimuth[r_idx,:], _ = aoa_capon(range_cube[r_idx],
                                              steering_vec)

    range_azimuth = np.log(np.abs(range_azimuth))

    return range_azimuth


@njit(cache=True)
def compute_range_azimuth_bartlett(radar_cube,
                                   angle_res=1,
                                   angle_range=90):
    """ Computes the range azimuth heatmap using the Bartlett method.
    """

    n_range_bins = radar_cube.shape[2]
    n_rx = radar_cube.shape[1]
    n_chirps = radar_cube.shape[0]
    n_angle_bins = (angle_range * 2) // angle_res + 1

    range_cube = np.zeros_like(radar_cube)
    with objmode(range_cube='complex128[:,:,:]'):
        range_cube = np.fft.fft(radar_cube, axis=2)
    # range_cube = np.transpose(range_cube, (2, 1, 0))
    range_cube = np.asarray(range_cube, dtype=np.complex64)

    _ , steering_vec = gen_steering_vec(angle_range, angle_res, n_rx)

    range_azimuth_cube = aoa_bartlett(steering_vec,
                                      range_cube,
                                      axis=1)

    range_azimuth = np.log(np.abs(np.sum(range_azimuth_cube, axis=0))).T

    return range_azimuth

@njit(cache=True)
def compute_altitude(radar_cube,
                     range_res,
                     range_bias,
                     window_len=3):
    """ Estimate range in the boresign direction. 
    """

    radar_cube_ = radar_cube - get_mean(radar_cube, axis=0)
    sum_rx = np.sum(radar_cube_, axis=1)

    range_response = np.zeros_like(sum_rx)
    with objmode(range_response='complex128[:,:]'):
        range_response = np.fft.fft(sum_rx, axis=1)
    range_response = np.asarray(range_response, dtype=np.complex64)

    range_response_1d = np.sum(np.abs(range_response), axis=0)
    range_response_1d_ = np.zeros(len(range_response_1d)-window_len)
    for r in range(len(range_response_1d_)):
        range_response_1d_[r] = np.sum(range_response_1d[r:r+window_len])
    # windowed_range_response_1d = np.array([np.sum(range_response_1d[r:r+window_len]) \
    #     for r in range(len(range_response_1d)-window_len)])

    range_bin = np.argmax(range_response_1d_)

    altitude = max(range_res*range_bin + range_bias, 0.0)
    
    return altitude


@njit(cache=True)
def compute_doppler(radar_cube,
                    velocity_max):
    """ Estimates doppler shift in boresight direction.

    Args:
        radar_cube (np.ndarray): Radar cube.
        velocity_max (float): Maximum velocity.

    Returns:
        float: Estimated velocity.
        doppler_response_1d (np.ndarray): 1d doppler heatmap. 

    """

    sum_rx = np.sum(radar_cube, axis=1)

    range_response = np.fft.fft(sum_rx, axis=1)
    doppler_range_response = np.abs(np.fft.fftshift(np.fft.fft(range_response, axis=0)))**2

    doppler_response_1d = np.sum(doppler_range_response, axis=1)
    doppler_response_1d = np.convolve(doppler_response_1d, np.ones(5), mode='same')

    velocity_bin = np.argmax(doppler_response_1d)-len(doppler_response_1d)//2

    return velocity_bin*(2*velocity_max/len(doppler_response_1d)), doppler_response_1d

@njit(cache=True)
def compute_doppler_azimuth(radar_cube,
                            angle_res=1,
                            angle_range=90,
                            range_initial_bin=0,
                            range_subsampling_factor=2):
    """ Computes the doppler azimuth heatmap.
    """


    n_chirps     = radar_cube.shape[0]
    n_rx         = radar_cube.shape[1]
    n_samples    = radar_cube.shape[2]
    n_angle_bins = (angle_range * 2) // angle_res + 1

    # Subsample range bins.
    radar_cube_ = radar_cube[:,:,range_initial_bin::range_subsampling_factor]
    radar_cube_ -= get_mean(radar_cube_, axis=0) 

    # Doppler processing.
    doppler_cube = np.zeros_like(radar_cube_)
    with objmode(doppler_cube='complex128[:,:,:]'):
        doppler_cube = np.fft.fft(radar_cube_, axis=0)
        doppler_cube = np.fft.fftshift(doppler_cube, axes=0)
    doppler_cube = np.asarray(doppler_cube, dtype=np.complex64)

    # Azimuth processing.
    _ , steering_vec = gen_steering_vec(angle_range, angle_res, n_rx)

    doppler_azimuth_cube = aoa_bartlett(steering_vec,
                                        doppler_cube,
                                        axis=1)
    # doppler_azimuth_cube = doppler_azimuth_cube[:,:,::5]
    doppler_azimuth_cube -= np.expand_dims(get_mean(doppler_azimuth_cube, axis=2), axis=2)

    doppler_azimuth = np.log(get_mean(np.abs(doppler_azimuth_cube)**2, axis=2))

    return doppler_azimuth

""" Helper functions. """

def normalize(data, min_val=None, max_val=None):
    """ Normalize floats to [0.0, 1.0].
    """
    if min_val is None:
        min_val = np.min(data)
    if max_val is None:
        max_val = np.max(data)
    img = (((data-min_val)/(max_val-min_val)).clip(0.0, 1.0)).astype(data.dtype)
    return img

