function [] = makeplot(var1,var2,tarray,zarray,animationSpeed,Tc,acr,energy,pos1,pos2,vel1,vel2,acc1,acc2)
%Animation speed:
move = animationSpeed;

time = tarray;
endtime = tarray(end);

%%%%%%%% 1st Subplot -- the pendulum animation %%%%%%%

subplot(3,4,[1 2 5 6 9 10]) %4x2 grid with animation taking the top panel of 3x2
axis equal
hold on

%Create pendulum link1 object:
width1 = acr.l1*0.05;
ydat1 = 0.5*width1*[-1 1 1 -1];
xdat1 = acr.l1*[0 0 1 1];
link1 = patch(xdat1,ydat1, [0 0 0 0],'r');

%Create pendulum link2 object:
width2 = acr.l2*0.05;
ydat2 = 0.5*width2*[-1 1 1 -1];
xdat2 = acr.l2*[0 0 1 1];
link2 = patch(xdat2,ydat2, [0 0 0 0],'r');
axis([-2.0 2.0 -2.6 2.6]); 
plot([0 0],[0 2.6],'--','Color','k','linewidth',0.2);
%Dots for the joints:
h1 = plot(0,0,'.k','MarkerSize',40); 
h2 = plot(0,0,'.k','MarkerSize',40); 

%%%%%%%% 2nd Subplot -- the system energy %%%%%%%
subplot(3,4,[3 4])
hold on
grid on
%Find the desired energy when the system is balanced.
energyPlot = plot(0,0,'k'); %Energy plot over time.
% plot([0,endtime],[desEnergy,desEnergy],'r'); %The line showing the target energy

axis([0,endtime,min(energy)-1,max(energy)+1]); %size to fit energy bounds and timescale

xlabel('Time (s)','FontSize',16)
ylabel('Energy (J)','FontSize',16)

hold off

%%%%%%%% 3rd Subplot -- the control torque %%%%%%%
subplot(3,4,[7 8])
hold on
grid on
torquePlot = plot(0,0,'k');
%axis([0,endtime,max(min(Tarray)-abs(min(Tarray))*0.1-0.1,-max(Tarray)),min(max(Tarray)*1.1,-min(Tarray)*1.1)]); %size to fit whatever output given
xlim([0,endtime])
ylim([min(Tc)-1, max(Tc)+1])
xlabel('Time (s)','FontSize',16)
ylabel('Torque (Nm)','FontSize',16)
hold off

%%%%%%%% 4rd Subplot -- vars %%%%%%%
subplot(3,4,[11 12])
hold on
grid on
    if strcmp(var1,'pos1') && strcmp(var2,'pos2')
        positionPlot1 = plot(0,0,'r');
        positionPlot2 = plot(0,0,'b');
        %axis([0,endtime,max(min(zarray(:,2))-abs(min(zarray(:,2)))*0.1-0.1,-max(zarray(:,2))),min(max(zarray(:,2))*1.1,-min(zarray(:,2))*1.1)]); %size to fit whatever output given
        xlim([0,endtime])
            if strcmp(acr.controller_type,'noncollocated')
                ylim([-20,20])
            else ylim([-10,10])
            end
        xlabel('Time (s)','FontSize',16)
        ylabel('Position','FontSize',16)
        hold off
    elseif strcmp(var1,'vel1') && strcmp(var2,'vel2')
        velocityPlot1 = plot(0,0,'r');
        velocityPlot2 = plot(0,0,'b');
        %axis([0,endtime,max(min(zarray(:,2))-abs(min(zarray(:,2)))*0.1-0.1,-max(zarray(:,2))),min(max(zarray(:,2))*1.1,-min(zarray(:,2))*1.1)]); %size to fit whatever output given
        xlim([0,endtime])
            if strcmp(acr.controller_type,'noncollocated')
                ylim([-20,20])
            else ylim([-10,10])
            end
        xlabel('Time (s)','FontSize',16)
        ylabel('Velocity','FontSize',16)
        hold off
    else 
        accelerationPlot1 = plot(0,0,'r');
        accelerationPlot2 = plot(0,0,'b');
        %axis([0,endtime,max(min(zarray(:,2))-abs(min(zarray(:,2)))*0.1-0.1,-max(zarray(:,2))),min(max(zarray(:,2))*1.1,-min(zarray(:,2))*1.1)]); %size to fit whatever output given
        xlim([0,endtime])
            if strcmp(acr.controller_type,'noncollocated')
                ylim([-20,20])
            else ylim([-10,10])
            end
        xlabel('Time (s)','FontSize',16)
        ylabel('Acceleration','FontSize',16)
        hold off
    end


%Make the whole window big for handy viewing:
set(gcf, 'units', 'inches', 'position', [5 5 18 9])

%Animation plot loop:
tic %Start the clock
while toc<endtime/move
    
    %If i close the figure, this prevents the error message
    if ishandle(1) == 0
        break;
    end
    
    tstar = move*toc; %Get the time (used during this entire iteration)
    
    zstar = interp1(time,zarray,tstar); %Interpolate data at this instant in time.
    
    %Rotation matrices to manipulate the vertices of the patch objects
    %using q1 and q2 from the output state vector.
    rot1 = [cos(zstar(1)), -sin(zstar(1)); sin(zstar(1)),cos(zstar(1))]*[xdat1;ydat1];
    set(link1,'xData',rot1(1,:))
    set(link1,'yData',rot1(2,:))
    
    rot2 = [cos(zstar(3)+zstar(1)), -sin(zstar(3)+zstar(1)); sin(zstar(3)+zstar(1)),cos(zstar(3)+zstar(1))]*[xdat2;ydat2];
    
    set(link2,'xData',rot2(1,:)+(rot1(1,3)+rot1(1,4))/2) %We want to add the midpoint of the far edge of the first link to all points in link 2.
    set(link2,'yData',rot2(2,:)+(rot1(2,3)+rot1(2,4))/2)
    
    %Change the joint dot location
    set(h2,'xData',(rot1(1,3)+rot1(1,4))/2)
    set(h2,'yData',(rot1(2,3)+rot1(2,4))/2)
    
    %Make the energy profile also plot out over time simultaneously.
    plotInd = time<tstar;
    set(energyPlot,'xData',time(plotInd)) %Plot all points that occur before our current time (not bothering with interpolation given the scale)
    set(energyPlot,'yData',energy(plotInd))
    
    %Make the torque profile also plot out over time simultaneously.
    set(torquePlot,'xData',time(plotInd)) %Plot all points that occur before our current time (not bothering with interpolation given the scale)
    set(torquePlot,'yData',Tc(plotInd))

    

if strcmp(var1,'pos1') && strcmp(var2,'pos2')
    %Make the torque profile also plot out over time simultaneously.
    set(positionPlot1,'xData',time(plotInd)) %Plot all points that occur before our current time (not bothering with interpolation given the scale)
    set(positionPlot1,'yData',pos1(plotInd))
    set(positionPlot2,'xData',time(plotInd)) %Plot all points that occur before our current time (not bothering with interpolation given the scale)
    set(positionPlot2,'yData',pos2(plotInd))
elseif strcmp(var1,'vel1') && strcmp(var2,'vel2')
    %Make the torque profile also plot out over time simultaneously.
    set(velocityPlot1,'xData',time(plotInd)) %Plot all points that occur before our current time (not bothering with interpolation given the scale)
    set(velocityPlot1,'yData',vel1(plotInd))
    set(velocityPlot2,'xData',time(plotInd)) %Plot all points that occur before our current time (not bothering with interpolation given the scale)
    set(velocityPlot2,'yData',vel2(plotInd))
else 
    %Make the torque profile also plot out over time simultaneously.
    set(accelerationPlot1,'xData',time(plotInd)) %Plot all points that occur before our current time (not bothering with interpolation given the scale)
    set(accelerationPlot1,'yData',acc1(plotInd))
    set(accelerationPlot2,'xData',time(plotInd)) %Plot all points that occur before our current time (not bothering with interpolation given the scale)
    set(accelerationPlot2,'yData',acc2(plotInd))
end

    legend('Link 1','Link 2'); 
    
    drawnow;
end

end