close all

%Playback speed
playback = animationSpeed;

time = time_array;
endtime = time_array(end);

%%%%%%%% 1st Subplot -- the pendulum animation %%%%%%%

subplot(3,4,[1 1.75 5 5.75 9 9.75]) %4x2 grid with animation taking the top panel of 3x2
axis equal
hold on

%Create pendulum link1 object:
width1 = acr.l1*0.05;
ydat1 = 0.5*width1*[-1 1 1 -1];
xdat1 = acr.l1*[0 0 1 1];
link1 = patch(xdat1,ydat1, [0 0 0 0],'b');

%Create pendulum link2 object:
width2 = acr.l2*0.05;
ydat2 = 0.5*width2*[-1 1 1 -1];
xdat2 = acr.l2*[0 0 1 1];
link2 = patch(xdat2,ydat2, [0 0 0 0],'b');
axis([-2.0 2.0 -2.6 2.6]); 
plot([0 0],[0 2.6],'--','Color','k','linewidth',0.2);

%Dots for the hinges:
h1 = plot(0,0,'.k','MarkerSize',40); %First link anchor
h2 = plot(0,0,'.k','MarkerSize',40); %link1 -> link2 hinge

hold off

%%%%%%%% 2nd Subplot -- the system energy %%%%%%%
subplot(3,4,[3 4])
hold on
grid on
%Find the desired energy when the system is balanced.
energyPlot = plot(0,0,'r'); %Energy plot over time.
% plot([0,endtime],[desEnergy,desEnergy],'r'); %The line showing the target energy

axis([0,endtime,min(energy)-1,max(energy) + 1]); %size to fit energy bounds and timescale

ylabel('Energy (J)','FontSize',16)

hold off

%%%%%%%% 3rd Subplot -- the control torque %%%%%%%
subplot(3,4,[7 8])
hold on
grid on
torquePlot = plot(0,0,'r');
%axis([0,endtime,max(min(time_array)-abs(min(time_array))*0.1-0.1,-max(time_array)),min(max(time_array)*1.1,-min(time_array)*1.1)]); %size to fit whatever output given
xlim([0,endtime])
ylim([min(torq)-1, max(torq)+1])
ylabel('Torque (Nm)','FontSize',16)
hold off

%%%%%%%% 4rd Subplot -- vars %%%%%%%
subplot(3,4,[11 12])
hold on
grid on

positionPlot1 = plot(0,0,'r');
positionPlot2 = plot(0,0,'b');
%axis([0,endtime,max(min(zarray(:,2))-abs(min(zarray(:,2)))*0.1-0.1,-max(zarray(:,2))),min(max(zarray(:,2))*1.1,-min(zarray(:,2))*1.1)]); %size to fit whatever output given
xlim([0,endtime])
ylim([min(min([pos1,pos2]))-100, max(max([pos1,pos2]))+100])

xlabel('Time (s)','FontSize',16)
ylabel('Position (deg/s)','FontSize',16)
hold off

%Make the whole window big for handy viewing:
set(gcf, 'units', 'inches', 'position', [5 5 18 9])

%Animation loop:
tic %Start the clock
while toc<endtime/playback

    tstar = playback*toc; %Get the time (used during this entire iteration)
    zstar = interp1(time,zarray,tstar); %Interpolate data at this instant in time.
    
    %Rotation matrices to manipulate the vertices
    %using q1 and q2 from the output state vector.
    rot1 = [cos(zstar(1)), -sin(zstar(1)); sin(zstar(1)),cos(zstar(1))]*[xdat1;ydat1];
    set(link1,'xData',rot1(1,:))
    set(link1,'yData',rot1(2,:))
    
    rot2 = [cos(zstar(3)+zstar(1)), -sin(zstar(3)+zstar(1)); sin(zstar(3)+zstar(1)),cos(zstar(3)+zstar(1))]*[xdat2;ydat2];
    
    set(link2,'xData',rot2(1,:)+(rot1(1,3)+rot1(1,4))/2) %We want to add the midpoint of the far edge of the first link to all points in link 2.
    set(link2,'yData',rot2(2,:)+(rot1(2,3)+rot1(2,4))/2)
    
    %Change the hinge dot location
    set(h2,'xData',(rot1(1,3)+rot1(1,4))/2)
    set(h2,'yData',(rot1(2,3)+rot1(2,4))/2)
    
    %Make the energy profile also plot out over time simultaneously.
    plotInd = time<tstar;
    set(energyPlot,'xData',time(plotInd)) %Plot all points that occur before our current time (not bothering with interpolation given the scale)
    set(energyPlot,'yData',energy(plotInd))
    
    %Make the power profile also plot out over time simultaneously.
    set(torquePlot,'xData',time(plotInd)) %Plot all points that occur before our current time (not bothering with interpolation given the scale)
    set(torquePlot,'yData',torq(plotInd))
    
    set(positionPlot1,'xData',time(plotInd)) %Plot all points that occur before our current time (not bothering with interpolation given the scale)
    set(positionPlot1,'yData',pos1(plotInd))
    set(positionPlot2,'xData',time(plotInd)) %Plot all points that occur before our current time (not bothering with interpolation given the scale)
    set(positionPlot2,'yData',pos2(plotInd))

    drawnow;
end

