function client()
%   provides a menu for accessing PIC32 motor control functions
%
%   client(port)
%
%   Input Arguments:
%       port - the name of the com port.  This should be the same as what
%               you use in screen or putty in quotes ' '
%
%   Example:
%       client('/dev/ttyUSB0') (Linux/Mac)
%       client('COM3') (PC)
%
%   For convenience, you may want to change this so that the port is hardcoded.

% Opening COM connection
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

port = '/dev/ttyUSB0';
fprintf('Opening port %s....\n',port);

% settings for opening the serial port. baud rate 230400, hardware flow control
% wait up to 120 seconds for data before timing out
mySerial = serial(port, 'BaudRate', 230400, 'FlowControl', 'hardware','Timeout',120);
% opens serial connection
fopen(mySerial);
% closes serial port when function exits
clean = onCleanup(@()fclose(mySerial));

has_quit = false;

% menu loop
while ~has_quit
    fprintf('PIC32 MOTOR DRIVER INTERFACE\n\n');
    % display the menu options; this list will grow
    fprintf('q: Quit    x: Add    c: Read Encoder    d: Read Encoder(degrees)\n');
    fprintf('e: Reset Encoder     r: Read Mode\n');
    fprintf('a: Read current sensor(ADC counts)      b: Read current sensor(mA)\n');
    fprintf('f: Set PWM(-100 to 100)    p: Unpower the motor\n');
    fprintf('g: Set Current Gains       h: Read Current Gains\n');
    fprintf('k: Test Current Gains\n');
    fprintf('i: Set Position Gains      j: Read Position Gains\n');
    fprintf('l: Hold Position\n');
    fprintf('m: Load Step Trajectory    n: Load Cubic Trajectory\n');
    fprintf('o: Execute Trajectory\n');

    % read the user choice
    selection = input('\nENTER COMMAND: ', 's');

    % send the command to the PIC32
    fprintf(mySerial,'%c\n',selection);

    % take the appropriate action
    switch selection
        case 'a'
            n = fscanf(mySerial,'%d');
            fprintf('Read: %d\n',n);
        case 'b'
            n = fscanf(mySerial,'%d');
            fprintf('Read: %d\n',n);
        case 'f'
            n = input('Enter PWM setting: ');
            fprintf(mySerial,'%d\n',n);
        case 'g'
            n1 = input('Enter P gain: ');
            n2 = input('Enter I gain: ');
            fprintf(mySerial,'%f %f\n',[n1 n2]);
        case 'h'
            n = fscanf(mySerial,'%f %f');
            fprintf('P: %f  I: %f\n',n);
        case 'i'
            n1 = input('Enter P gain: ');
            n2 = input('Enter D gain: ');
            n3 = input('Enter I gain: ');
            fprintf(mySerial,'%f %f %f\n',[n1 n2 n3]);
        case 'j'
            n = fscanf(mySerial,'%f %f %f');
            fprintf('P: %f  D: %f  I: %f\n',n);
        case 'k'
            read_plot_matrix(mySerial);
        case 'l'
            n = input('Enter Angle (deg): ');
            fprintf(mySerial, '%d\n',n);
        case 'm'
            n = input('Enter [Time,Position;] Array: ');    %n is input array
            n = genRef(n,'step');                           %n:trajectory array
            %send number of samples to pic
            m = length(n);
            fprintf(mySerial,'%d\n',m);
            %send samples (just reference positions)
            for i=1:m
                fprintf(mySerial,'%f\n',n(i));
            end
        case 'n'
            n = input('Enter [Time,Position;] Array: ');    %n is input array
            n = genRef(n,'cubic');                          %n:trajectory array
            %send number of samples to pic
            m = length(n);
            fprintf(mySerial,'%d\n',m);
            %send samples (just reference positions)
            for i=1:m
                fprintf(mySerial,'%f\n',n(i));
            end
            %for i=1:m
            %    n = fscanf(mySerial,'%f');
            %    fprintf('%f\n',n);
            %end
        case 'o'
            fprintf('Tracking\n');

            read_plot_matrix_position(mySerial);

        case 'p'
            fprintf('Stopping');
        case 'x'
            n1 = input('Enter a number: ');
            n2 = input('Enter another number: ');
            fprintf(mySerial, '%d %d\n',[n1 n2]);
            n = fscanf(mySerial,'%d');
            fprintf('Read: %d\n',n);
        case 'c'
            n = fscanf(mySerial,'%d');
            fprintf('Read: %d\n',n);
        case 'e'
            n = fscanf(mySerial,'%d');
            fprintf('Reset to: %d\n',n);
        case 'd'
            n = fscanf(mySerial,'%d');
            fprintf('Read: %d\n',n);
        case 'r'
            str = fscanf(mySerial,'%s');
            fprintf('Mode: %s\n',str);
        case 'q'
            has_quit = true;             % exit client
        otherwise
            fprintf('Invalid Selection %c\n', selection);
    end
end

end
