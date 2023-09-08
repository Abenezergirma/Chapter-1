function dydt = Octocopter(t,y,actions)

            Ixx = 12;
            Iyy = 12;
            Izz = 12; % 
            l = 1; % arm length
            mt = 15; % total mass of the UAV

            g = 9.81;

            %%% control actions %%%
            n = length(actions(:,1));

            y = reshape(y, n, []);

            [T, tauPhi, tauTheta, tauPsi] = deal(actions(:,1), actions(:,2), actions(:,3), actions(:,4));

            [x, y, z, xDot, yDot, zDot, phi, theta, psi, p, q, r] = deal(y(:,1), ...
                y(:,2), y(:,3), y(:,4), y(:,5), y(:,6), y(:,7), y(:,8), y(:,9), y(:,10), y(:,11), y(:,12));
            % assuming no wind

            xRate = xDot;
            yRate = yDot;
            zRate = zDot;
            xDRate = (sin(theta).*cos(psi).*cos(phi) + sin(phi).*sin(psi)).*T/mt;
            yDRate = (sin(theta).*sin(psi).*cos(phi) - sin(phi).*sin(psi)).*T/mt;
            zDRate = -g + (cos(psi).*cos(theta)).*T/mt;
            phiRate = p + q.*sin(phi).*tan(theta) + r.*cos(phi).*tan(theta);
            thetaRate = q.*cos(phi) - r.*sin(phi);
            psiRate = q.*(sin(phi)./cos(theta)) + r.*(cos(phi)./cos(theta));
            pRate = ((Iyy - Izz)/Ixx).*q.*r + (l/Ixx).*tauPhi;
            qRate = ((Izz - Ixx)/Iyy).*p.*r + (l/Iyy).*tauTheta;
            rRate = ((Ixx - Iyy)/Izz).*q.*r + (l/Izz).*tauPsi;

            dydt = [xRate; yRate; zRate; xDRate; yDRate; zDRate; phiRate; thetaRate;psiRate;pRate;qRate;rRate  ];

        end