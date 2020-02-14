import numpy as np
import pdb

class Resampling:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 4.3]
    """

    def __init__(self):
        """
        TODO : Initialize resampling process parameters here
        """

    def multinomial_sampler(self, X_bar):

        """
        param[in] X_bar : [num_particles x 4] sized array containing [x, y, theta, wt] values for all particles
        param[out] X_bar_resampled : [num_particles x 4] sized array containing [x, y, theta, wt] values for resampled set of particles
        """

        """
        TODO : Add your code here
        """
        X_bar_resampled = np.zeros(X_bar.shape)
        weights_sum = np.sum(X_bar[:,3])
        weights = X_bar[:,3] / weights_sum
        samples = np.random.multinomial(len(X_bar), weights, size=1)
        counter = 0
        for i, s in enumerate(samples[0]):
            for j in range(s):
                X_bar_resampled[counter] = X_bar[i]
                counter += 1
        return X_bar_resampled

    def low_variance_sampler(self, X_bar):

        """
        param[in] X_bar : [num_particles x 4] sized array containing [x, y, theta, wt] values for all particles
        param[out] X_bar_resampled : [num_particles x 4] sized array containing [x, y, theta, wt] values for resampled set of particles
        """

        """
        TODO : Add your code here
        """
        X_bar[:,3] = X_bar[:,3]/np.sum(X_bar[:,3])
        X_bar_resampled = list()
        M = X_bar.shape[0]
        r = np.random.uniform(0, 1/M)
        c = X_bar[0,3]

        i = 0
        for m in range(1, M+1):
            u = r + (m-1)*1/M
            while u > c:
                i+=1
                c+= X_bar[i,3]
            X_bar_resampled.append(X_bar[i])
        X_bar_resampled = np.array(X_bar_resampled)
        return X_bar_resampled 

if __name__ == "__main__":
    pass
