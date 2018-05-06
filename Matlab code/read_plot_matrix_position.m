function data = read_plot_matrix(mySerial)
  nsamples = fscanf(mySerial,'%d');       % first get the number of samples being sent
  data = zeros(nsamples,2);               % two values per sample:  ref and actual
  for i=1:nsamples
    data(i,:) = fscanf(mySerial,'%f %f'); % read in data from PIC32; assume floats, in degrees
    times(i) = (i-1)*.005;                 % 5 ms between samples
  end
  if nsamples > 1
    stairs(times,data(:,1:2));            % plot the reference and actual
  else
    fprintf('Only 1 sample received\n')
    disp(data);
  end
  % compute the average error
  score = mean(abs(data(:,1)-data(:,2)));
  fprintf('\nAverage error: %5.1f Degrees\n',score);
  title(sprintf('Average error: %5.1f Degrees',score));
  ylabel('Degrees');
  xlabel('Time (ms)');
end
