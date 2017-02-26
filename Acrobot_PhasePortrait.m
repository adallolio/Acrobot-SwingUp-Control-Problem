function [] = Acrobot_PhasePortrait(m1,m2,I1,I2,lc1,lc2,l1,l2,g)
    if strcmp(controller_type,'noncollocated') 
        % NON-COLLOCATED case
        
        % Constants
        a1 = m2*lc2^2+I2;
        a2 = m2*l1*lc2;
        a3 = m2*l1*lc2;
        a4 = m2*lc2*g;
        % Defines the differential equation
        f = @(t,X) [X(2); (a3*sin(X(1))*X(2)^2+a4*sin(X(1)))/(a1+a2*cos(X(1)))];

        % Defines the ranges for each variable
        y1 = linspace(-2*pi,2*pi,80);
        y2 = linspace(-6*pi,6*pi,80);
        
         % Creates the grid
        [x,y] = meshgrid(y1,y2);
        u = zeros(size(x));
        v = zeros(size(x));

        t=0; % we want the derivatives at each point at t=0, i.e. the starting time
        for i = 1:numel(x)
            Yprime = f(t,[x(i); y(i)]);
            u(i) = Yprime(1);
            v(i) = Yprime(2);
        end
        hold on
        figure(gcf)
        %hold on
        quiver(x,y,u,v,'r'); 
        xlabel('X_1')
        ylabel('X_2')
        %axis tight equal;

        %x1 = [6.28,-6.28, -6.28, -1.914, 6.28, 2.282];
        %x2 = [-8.93, 8.92, 6.54, 9.79, -6.54, -11.31];

        x1 = [-6.28, 6.28, -6.28, 6.28, -6.28, 6.28, -6.28, 6.28, -6.28, 6.28];
        x2 = [5.01, -5.01, 5.58, -5.58, 6.32, -6.32, 7.04, -7.04, 7.75, -7.75];
        time = [1.7, 1.7, 1.5, 1.5, 1.0, 1.0, 1.25, 1.25, 1.1, 1.1];
        for i = 1:length(x1)
            [~,ys] = ode45(f,[0,time(i)],[x1(i);x2(i)]);
            plot(ys(:,1),ys(:,2))
            %plot(ys(1,1),ys(1,2),'bo') % starting point
            %plot(ys(end,1),ys(end,2),'ks') % ending point
        end

    hold off
        
    elseif strcmp(controller_type,'collocated') 

    end
  
    
    
end

