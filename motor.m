function robot=motor(robot,whichmotor,operation,value)
assert(~isequal(operation,'power') || (value>=0 && value<=robot.motor.power.max),'Invalid power.')
assert(~isequal(operation,'dir') || (value==0 || value==1),'Invalid direction.')
if isequal(whichmotor,'left') && isequal(operation,'dir')
    value=double(~value);
end
if isequal(operation,'dir') && value==0
    value=-1;
end
switch whichmotor
    case 'right'
        switch operation
            case 'power'
                robot.motor.power.right=value;
                RPMs=polyval(robot.motor.poly,robot.motor.power.right);
                if RPMs<=0
                    %warning('Right-motor power too low. Wheel RPM=0.')
                    RPMs=0;
                end
                Vs=(robot.motor.wheeldia/2)*((RPMs/60)*2*pi);
                robot.kinematics.V.right=Vs;
            case 'dir'
                robot.motor.dir.right=value;
        end
    case 'left'
        switch operation
            case 'power'
                robot.motor.power.left=value;
                RPMs=polyval(robot.motor.poly,robot.motor.power.left);
                if RPMs<=0
                    %warning('Left-motor power too low. Wheel RPM=0.')
                    RPMs=0;
                end
                Vs=(robot.motor.wheeldia/2)*((RPMs/60)*2*pi);
                robot.kinematics.V.left=Vs;
            case 'dir'
                robot.motor.dir.left=value;
        end
end
end