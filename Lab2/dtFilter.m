function retval = dtFilter(dt)
% dt band filter. 
% Apply a bandpass filter to the input data. 
ref = 1E-2; 
threshold = 0.5; 
if dt < (1-threshold)*ref || dt > (1+threshold)*ref
    retval = ref; 
else
    retval = dt; 
end
