function simout = reducedOrderSim(pushParam,simParam,simTime)


% load push data
pushForce = pushParam.force;
pushDur   = pushParam.duration;
pushOnset = pushParam.onSet;

model = simParam.model;
nlobj = simParam.nlobj;

Ts = nlobj.Ts;
p  = nlobj.PredictionHorizon;


modelProperties     = simParam.nlParameters.modelProperties;
stateLim            = simParam.nlParameters.stateLim;
controlLim          = simParam.nlParameters.controlLim;
stateWeight         = simParam.nlParameters.stateWeight;
controlWeight       = simParam.nlParameters.controlWeight;
controlRateWeight   = simParam.nlParameters.controlRateWeight;

z0 = modelProperties(3);


N = 1000;

time = zeros(N,1);

switch model
    
    %% VHIPPFW
    case 4
        % initial condition
        x = [0; 0; z0 ;0; 0; 0];
        % set initial controls
        nloptions = nlmpcmoveopt;
        nloptions.MVTarget = [0 0 0];
        
        % desired State
        yref = [zeros(1,p);
               zeros(1,p);
               z0*ones(1,p);
               zeros(1,p);
               zeros(1,p);
               zeros(1,p)];    
           
       
        X = zeros(N,6);
        U = zeros(N,3);
        
    case 3 
        % initial condition
        x = [0; z0; 0; 0];
        % set initial controls
        nloptions = nlmpcmoveopt;
        nloptions.MVTarget = [0 0];
        
        % desired state
        yref = [zeros(1,p);
               z0*ones(1,p);
               zeros(1,p);
               zeros(1,p)];    
         
        X = zeros(N,4);
        U = zeros(N,2);
           
    case 2 
        % initial condition
        x = [0; 0; 0; 0];
        % set initial controls
        nloptions = nlmpcmoveopt;
        nloptions.MVTarget = [0 0];
        
        % desired state
        yref = zeros(4,p);         
        
        X = zeros(N,4);
        U = zeros(N,2);
    case 1
        
        % initial condition
        x = [0; 0];
        % set initial controls
        nloptions = nlmpcmoveopt;
        nloptions.MVTarget = 0;
        
        % desired state
        yref = zeros(2,p);
        
        X = zeros(N,2);
        U = zeros(N,1);

        
end        
        nloptions.Parameters = {modelProperties,stateLim,controlLim,stateWeight,controlWeight,controlRateWeight};
        mv = nloptions.MVTarget;

        hbar = waitbar(0,'Single Simulation Progress');
        xHistory = x';
        lastMV = mv;
        uHistory = lastMV;
        

       
tic        
        %% simulate between [0-pushOnset]
        for k = 1:1:floor(pushOnset/Ts)
            
            
            % Compute the control moves with reference previewing.
            xk = xHistory;
            
            
            [uk,nloptions] = nlmpcmove(nlobj,xk,lastMV,yref',[],nloptions);
            uHistory = uk';
            lastMV = uk;

            % Update states
            
            
            ODEFUN = odeFunGen(model,uk,0);

            [TOUT,YOUT] = ode45(ODEFUN,[(k-1)*Ts k*Ts], xHistory');
            
            
            xHistory = YOUT(end,:);
            
            waitbar(k*Ts/simTime,hbar);
            
            UOUT = ones(length(TOUT),size(uk,1))*diag(uk);
            
            
            [time, ind] = appendData(time,TOUT);
            X = appendData(X,YOUT,ind);
            U = appendData(U,UOUT,ind);

        end

        % simulate between sample and pushOnset if needed
        if rem(pushOnset,Ts) ~= 0
            k = k + 1; % k such Ts*k > pushOnset
            
            
            % Compute the control moves with reference previewing.
            xk = xHistory;
            
            
            [uk,nloptions] = nlmpcmove(nlobj,xk,lastMV,yref',[],nloptions);
            uHistory = uk';
            lastMV = uk;

            % Update states
 
            ODEFUN = odeFunGen(model,uk,0);

            [TOUT,YOUT] = ode45(ODEFUN,[(k-1)*Ts pushOnset], xHistory');
            
            
            xHistory = YOUT(end,:);
            
            UOUT = ones(length(TOUT),size(uk,1))*diag(uk);

            [time, ind] = appendData(time,TOUT);
            X = appendData(X,YOUT,ind);
            U = appendData(U,UOUT,ind);
            
            

        
            
%% simulate between pushOnset and next sample time


            ODEFUN = odeFunGen(model,uk,pushForce);
             
            [TOUT,YOUT] = ode45(ODEFUN,[pushOnset, k*Ts], xHistory');
             
            xHistory = YOUT(end,:);
            
            UOUT = ones(length(TOUT),size(uk,1))*diag(uk);
            
            
            [time, ind] = appendData(time,TOUT);
            X = appendData(X,YOUT,ind);
            U = appendData(U,UOUT,ind);
            
             waitbar(k*Ts/simTime,hbar);
        end
        
%% simulate between first sample during push ineterval to last        
        for k =  ceil(pushOnset/Ts)+ 1 :1: floor((pushOnset + pushDur)/Ts)
            
            % Compute the control moves with reference previewing.
            xk = xHistory;
            
            
            [uk,nloptions] = nlmpcmove(nlobj,xk,lastMV,yref',[],nloptions);
            uHistory = uk';
            lastMV = uk;

            % Update states
            
            ODEFUN = odeFunGen(model,uk,pushForce);
            
            [TOUT,YOUT] = ode45(ODEFUN,[(k-1)*Ts k*Ts], xHistory');
            xHistory = YOUT(end,:);
            waitbar(k*Ts/simTime,hbar);
            
            UOUT = ones(length(TOUT),size(uk,1))*diag(uk);
            
            [time,ind] = appendData(time,TOUT);
            X = appendData(X,YOUT,ind);
            U = appendData(U,UOUT,ind);
            
            
        end
        
        % simulate after push to next sample point if needed
        if (rem((pushOnset + pushDur),Ts) ~= 0)
            k = k + 1; % k such Ts*k > pushOnset+pushDur
            
            
            % Compute the control moves with reference previewing.
            xk = xHistory;
            
            
            [uk,nloptions] = nlmpcmove(nlobj,xk,lastMV,yref',[],nloptions);
            uHistory = uk';
            lastMV = uk;

            % Update states
 
            ODEFUN = odeFunGen(model,uk,pushForce);

            [TOUT,YOUT] = ode45(ODEFUN,[(k-1)*Ts pushOnset+pushDur], xHistory');
            
            
            xHistory = YOUT(end,:);
            
            UOUT = ones(length(TOUT),size(uk,1))*diag(uk);

            [time, ind] = appendData(time,TOUT);
            X = appendData(X,YOUT,ind);
            U = appendData(U,UOUT,ind);
            
            

        
            
%% simulate after pushEnds to next sample time


            ODEFUN = odeFunGen(model,uk,0);
             
            [TOUT,YOUT] = ode45(ODEFUN,[pushOnset+pushDur, k*Ts], xHistory');
             
            xHistory = YOUT(end,:);
            
            
            UOUT = ones(length(TOUT),size(uk,1))*diag(uk);
            
            
            [time, ind] = appendData(time,TOUT);
            X = appendData(X,YOUT,ind);
            U = appendData(U,UOUT,ind);
            
             waitbar(k*Ts/simTime,hbar);        
        
        
        
        end
        
        for k =  ceil((pushOnset + pushDur)/Ts):1: simTime/Ts 

            % Compute the control moves with reference previewing.
            xk = xHistory;
            
            
            [uk,nloptions] = nlmpcmove(nlobj,xk,lastMV,yref',[],nloptions);
            uHistory = uk';
            lastMV = uk;

            % Update states
            
            ODEFUN = odeFunGen(model,uk,0);
            
            [TOUT,YOUT] = ode45(ODEFUN,[(k-1)*Ts k*Ts], xHistory');
            
            if YOUT(end,1) > 0.266
                
                disp('fall detected')
                
                break
                
                
            end
            
            xHistory = YOUT(end,:);
            waitbar(k*Ts/simTime,hbar);
            
            
            
            UOUT = ones(length(TOUT),size(uk,1))*diag(uk);
            
            
            X = appendData(X,YOUT);
            U = appendData(U,UOUT);
            time = appendData(time,TOUT);
            
            

            
        end        
        solveTime = toc;
        close(hbar)
        
        
    simout.state = trimData(X);
    simout.control = trimData(U);
    simout.time = trimData(time);
    simout.solveTime = solveTime;
    
    
    switch model
        
        case 4
           simout.controlContribution = controlContribution(U,controlLim,Ts);
           simout.capturePoint.data = capturePoint(X(:,1),X(:,4),X(:,3));
        case 3
           simout.controlContribution = controlContribution(U,controlLim([1,3]),Ts);
           simout.capturePoint.data = capturePoint(X(:,1),X(:,3),X(:,2));
        case 2
           simout.controlContribution = controlContribution(U,controlLim([1,2]),Ts);
           simout.capturePoint.data = capturePoint(X(:,1),X(:,3),z0); 
        case 1
           simout.controlContribution = controlContribution(U,controlLim(1),Ts);
           simout.capturePoint.data = capturePoint(X(:,1),X(:,2),z0); 
        
        
    end
    
    simout.capturePoint.max = max(abs(simout.capturePoint.data));
    
    simout.pushParam = struct('force',pushForce,'duration',pushDur,'onSet',pushOnset);
    
    
    
    
    % utility functions
    function [relCont] = controlContribution(U,uMax,Ts)

        totalCost = 0;
        compCost  = zeros(1,size(U,2));


        for i = 1:size(U,2)

            compCost(i) = Ts*sum(U(:,i).^2*uMax(i)^(-2));
            totalCost = totalCost + compCost(i);

        end

        relCont = compCost/totalCost;

    end


    function cp = capturePoint(x,xdot,z)
        
        cp = x + xdot./sqrt((9.81)./z);
        
    end

    function  f = odeFunGen(model,uk,pushForce)
        
        switch model
            
            case 4
                f = @(t,xk) pushVHIPPFW(xk,uk,pushForce,modelProperties);
            case 3
                f = @(t,xk) pushVHIP(xk,uk,pushForce,modelProperties);
            case 2
                f = @(t,xk) pushLIPPFW(xk,uk,pushForce,modelProperties);
            case 1
                f = @(t,xk) pushLIP(xk,uk,pushForce,modelProperties);       
        end
        
        
    end

    
end

function newData = trimData(data)

    sz = size(data);
    
    
    k = find(~ismember(data,zeros(1,sz(2)),'rows'),1,'last');
    
    if  k == 1 || isempty(k)
        
        newData = data;
    else
        
        newData = data(1:k,:);
    end
 

end





