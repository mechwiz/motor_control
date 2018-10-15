function client(port)
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
    fprintf('\ta: Read current sensor (ADC counts)\tb: Read current sensor (mA)\n\tc: Read encoder (counts)\t\td: Read encoder (deg)\n\te: Reset encoder\t\t\tf: Set PWM (-100 to 100)\n\tg: Set current gains\t\t\th: Get current gains\n\ti: Set position gains\t\t\tj: Get position gains\n\tk: Test current control\t\t\tl: Go to angle (deg)\n\tm: Load step trajectory\t\t\tn: Load cubic trajectory\n\to: Execute trajectory\t\t\tp: Unpower the motor\n\tq: Quit\t\t\t\t\tr: Get mode\n');
    % read the user's choice
    selection = input('\nENTER COMMAND: ', 's');
     
    % send the command to the PIC32
    fprintf(mySerial,'%c\n',selection);
    
    % take the appropriate action
    switch selection
        case 'a'                         % example operation
            n = fscanf(mySerial,'%d');   % get angle counts back
            fprintf('The motor current is %d ADC counts.\n\n',n);
            
        case 'b'                         % example operation
            n = fscanf(mySerial,'%d');   % get angle counts back
            fprintf('The motor current is %d mA.\n\n',n);
        
        case 'c'                         % example operation
            n = fscanf(mySerial,'%d');   % get angle counts back
            fprintf('The motor angle is %d counts.\n\n',n);
        
        case 'd'                         % example operation
            n = fscanf(mySerial,'%f');   % get angle in degrees back
            fprintf('The motor angle is %.1f degrees.\n\n',n);     % print it to the screen
            
        case 'e'                         % example operation
            n = fscanf(mySerial,'%d');   % get angle in degrees back
            fprintf('The motor encoder is now reset to %d counts.\n\n',n);     % print it to the screen
          
        case 'f'                         % example operation
            n = input('What PWM value would you like [-100 to 100]? ');
            fprintf(mySerial, '%d\n',n);
            n = fscanf(mySerial,'%d');   % get PWM duty cycle back
            if sign(n) > 0
                dir = 'counterclockwise';
            elseif sign(n) < 0
                dir = 'clockwise';
            else
                dir = 'zero';
            end
            
            fprintf('PWM has been set to %d%% in the %s direction.\n\n',abs(n),dir);     % print it to the screen
                
        case 'g'                                                 % example operation
            kp = input('Enter your desired Kp current gain [recommended: 4.76]: ');
            ki = input('Enter your desired Ki current gain [recommended: 0.32]: ');
            kd = input('Enter your desired Kd current gain [recommended: 1.50]: ');
            fprintf(mySerial, '%f,%f,%f\n',[kp,ki,kd]);            % send the numbers
            n = fscanf(mySerial,'%f,%f,%f');                           % get the added numbers back
            fprintf('Sending Kp = %.2f, Ki = %.2f, and Kd = %.2f to the current controller.\n\n',[n(1),n(2),n(3)]);

        case 'h'                                                 % example operation
            n = fscanf(mySerial,'%f,%f,%f');                           % get the added numbers back
            fprintf('The current controller is using Kp = %.2f, Ki = %.2f, and Kd = %.2f.\n\n',[n(1),n(2),n(3)]);
            
        case 'i'                                                 % example operation
            kp = input('Enter your desired Kp current gain [recommended: 4.76]: ');
            ki = input('Enter your desired Ki current gain [recommended: 0.32]: ');
            kd = input('Enter your desired Kd current gain [recommended: 10.63]: ');
            fprintf(mySerial, '%f,%f,%f\n',[kp,ki,kd]);            % send the numbers
            n = fscanf(mySerial,'%f,%f,%f');                           % get the added numbers back
            fprintf('Sending Kp = %.2f, Ki = %.2f, and Kd = %.2f to the position controller.\n\n',[n(1),n(2),n(3)]);

        case 'j'                                                 % example operation
            n = fscanf(mySerial,'%f,%f,%f');                           % get the added numbers back
            fprintf('The position controller is using Kp = %.2f, Ki = %.2f, and Kd = %.2f.\n\n',[n(1),n(2),n(3)]);
        
        case 'k'                                                 % example operation
            fprintf('Testing the gains');
            read_plot_matrix(mySerial);
            
        case 'l'                                                 % example operation
            n = input('Enter the desired motor angle in degrees: ');
            fprintf(mySerial, '%f\n',n);            % send the numbers
            n = fscanf(mySerial,'%f');                           % get the added numbers back
            fprintf('Motor moving to %.1f degrees.\n\n',n);
            
        case 'm'
            n = input('Enter step trajectory, in sec and degrees [time1, ang1; time2, ang2; ...]: ');
            fprintf('Plotting the desired trajectory and sending to the PIC32 ... ');
            traj = genRef(n,'step');
            fprintf(mySerial,'%d\n',length(traj));
            for i=1:length(traj)
                fprintf(mySerial, '%f\n',traj(i));
            end
            n = fscanf(mySerial,'%d');
            fprintf('Plotting completed. %d samples loaded.\n\n',n);
            
        case 'n'
            n = input('Enter cubic trajectory, in sec and degrees [time1, ang1; time2, ang2; ...]: ');
            fprintf('Plotting the desired trajectory and sending to the PIC32 ... ');
            traj = genRef(n,'cubic');
            fprintf(mySerial,'%d\n',length(traj));
            for i=1:length(traj)
                fprintf(mySerial, '%f\n',traj(i));
            end
            n = fscanf(mySerial,'%d');
            fprintf('Plotting completed. %d samples loaded.\n\n',n);
            
        case 'o'                                                 % example operation
            fprintf('Executing Trajectory ... ');
            read_traj_response(mySerial);
            fprintf('Executing Done\n\n');
            
        case 'p'                         % example operation
            n = fscanf(mySerial,'%d');   % get angle in degrees back
            fprintf('The motor has been unpowered.\n\n');     % print it to the screen
            
        case 'r'                         % example operation
            n = fscanf(mySerial,'%s');   % get angle in degrees back
            fprintf('The PIC32 controller mode is currently %s.\n\n',n);     % print it to the screen
        
        case 'q'
            has_quit = true;             % exit client
        otherwise
            fprintf('Invalid Selection %c\n', selection);
    end
end

end
