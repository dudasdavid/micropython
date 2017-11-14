def filter2(ylast, x, alpha = 0.2):
    y = (alpha * x) + (1.0 - alpha) * ylast
    return y

def LPF(ylast, x, CUTOFF, SAMPLE_RATE):
    RC = 1.0/(CUTOFF*2*3.14)
    dt = 1.0/SAMPLE_RATE
    alpha = dt/(RC+dt)
    y = ylast + alpha * ( x - ylast )
    return y

def HPF(ylast, x, xlast, CUTOFF, SAMPLE_RATE):
    RC = 1.0/(CUTOFF*2*3.14)
    dt = 1.0/SAMPLE_RATE
    alpha = RC/(RC+dt)
    y = alpha * ( ylast + x - xlast)
    return y

'''
float kalman_single(float z, float measure_noise, float process_noise){
 const float R=measure_noise*measure_noise;
 const float Q=process_noise*process_noise; 
 static float x_hat,P;
 float P_,K;
 
 /********* noi suy kalman ***************/
    P_ = P + Q;                     // P_ = A*P*A' + Q;
    K = P_/(P_ + R);                // K = P_*H'*inv(H*P_*H' + R);
    x_hat = x_hat + K*(z - x_hat);  // x_hat = x_hat + K*(z - H*x_hat);
    P = ( 1 - K)*P_ ;               // P = ( 1 - K*H)*P_ ;
 /****************************************/ 
 return x_hat;
}
'''
