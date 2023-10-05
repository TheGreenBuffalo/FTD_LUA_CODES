-- By TheGreenBuffalo & Kuat Yards 9/6/2020-8/7/2020
-- Edit: 4th Order TheGreenBuffalo 23/09/2023 (and fixed crash when no AI)

counter = -1;                                  -- effectively value of 40Hz clock
clock = -1;                                    -- increments every physics step [40 Hz]

freq = 10;                                      -- ticks after which to take target data [>10,<=40]
snap = 10;                                     -- number of 'Snapshots' to keep
G = 9.8;                                        -- Should be Gravity (M/S^2)

TGTD = {};                                     -- Contains data at every 'Snapshot'
TGTC = {};                                     -- Contains target centroids from each 'Snapshot'
TGTS = {};                                     -- Contains target velocity and acceleration


function Update(I)

clock = clock + 1;
counter = counter + 1;                         -- increment on code execution (40Hz)

if counter == freq-1 then
   I:ClearLogs();                              -- Only For Debugging
   counter = 0;                                -- Resets Counter every second
end

-- Obtain Target Data
if counter == 0 then                           -- Obtain data once every second
   I:Log(clock);

   POS = {};                                   -- Array storing all Target and Aimpoint info
   NumM = I:GetNumberOfMainframes();           -- Used to find targeting Mainframes
   dat1 = false;                               -- Used to initiate while loop
   AIidx = -1;                                 -- Used to track mainframe index
 
   while dat1 == false do                      -- Search for Usable Mainframe
      AIidx = AIidx+1;                         -- Increment Mainframe Index
      var = I:Component_GetBlockInfo(26,AIidx);-- Get Block info on Mainframe

      if var.CustomName == 'LUAAI' then        -- If Mainframe is part of system...
         pos = I:GetTargetPositionInfo(AIidx,0);-- Get Real position of Origin Block
         POS[1] = pos;                         -- Store in 'POS' Array as first element
         I:Log('origin position         = '..pos.Position[1]..' '..pos.Position[2]..' '..pos.Position[3]);
         dat1 = true;                          -- end while loop
      end
      if AIidx == NumM and dat1 == false then    -- If none of the AIs are set correctly, reset so we don't crash
         clock = -1;
         count = -1;
	  end
   end

   ele = 1;
   for idx = 0,NumM-1 do                       -- Get Aimpoint of every targeting AI, all of which
      var = I:Component_GetBlockInfo(26,idx);  -- should be within ship sized bubble offset from
      if var.CustomName == 'LUAAI' then        -- the real ship
         pos = I:GetTargetInfo(idx,0);
         ele = ele+1;                          -- Stores length of POS array
         POS[ele] = pos;                       -- Store Aimpoint of each AI in array
      end
   end
   I:Log('AI targeted position = '..pos.Position[1]..' '..pos.Position[2]..' '..pos.Position[3])

   for idx = 1,snap-1 do
      TGTD[idx] = TGTD[idx+1];                 -- Shifts entire array down one element
      TGTC[idx] = TGTC[idx+1];
   end
   
   TGTD[snap] = POS;                           -- Places most recent 'Snapshot' in element 10

-- Calculate Aim Point
   -- Calculate Centroids

   tgtc = {};                                  -- Stores centroid of most recent snapshot

   cenx = 0;                                   -- Centroid x co-ordinate
   ceny = 0;                                   -- Centroid y co-ordinate
   cenz = 0;                                   -- Centroid z co-ordinate

   den = 0;                                    -- Denominator to find mean of Aimpoints

   for idx = 2,ele do                          -- Takes Summation of aimpoints
      cenx = TGTD[snap][idx].AimPointPosition[1]+cenx;
      ceny = TGTD[snap][idx].AimPointPosition[2]+ceny;
      cenz = TGTD[snap][idx].AimPointPosition[3]+cenz;
      den = den+1;                             -- Increments denominator
   end

   cenx = cenx/den;                            -- Summation/#summed values = mean
   ceny = ceny/den;
   cenz = cenz/den;

   tgtc[1] = cenx;                             -- place values into tgtc vector
   tgtc[2] = ceny;                             
   tgtc[3] = cenz;

   TGTC[snap] = tgtc;                          -- Save this centroid vector for later
   TGTC[snap][1] = TGTD[snap][1].Position[1] - TGTC[snap][1];
   TGTC[snap][2] = TGTD[snap][1].Position[2] - TGTC[snap][2];
   TGTC[snap][3] = TGTD[snap][1].Position[3] - TGTC[snap][3];
   
   TGTC[snap][1] = TGTD[snap][2].Position[1] + TGTC[snap][1];
   TGTC[snap][2] = TGTD[snap][2].Position[2] + TGTC[snap][2];
   TGTC[snap][3] = TGTD[snap][2].Position[3] + TGTC[snap][3];

   I:Log('Average Aimpoint    = '..TGTC[snap][1]..' '..TGTC[snap][2]..' '..TGTC[snap][3]);-- Used for Debugging
end

   -- Predict Position of True Origin
      -- High order extrapolations of the form seen below historically have been known to
      -- wildly differ from the actual data when noise is introduced. In other words, if I
      -- were to use a 7th order extrapolation to find the target moving with a noisy parabolic
      -- flight path, I would likely just miss completely. As such, this code uses a second order
      -- method to extrapolate the position of the true origin

      -- As we are doing a second order position estimate, we can assume constant acceleration.
      -- Therefore, if we find the most recent velocity and acceleration, we have all the info
      -- we need to predict the next position

      -- These values will be found numerically using a backwards finite difference method as
      -- neither the TargetInfo or TargetPositionInfo objects contain the real velocity or
      -- any acceleration value of the target.

if clock > freq*5 then
   if counter == 0 then
   I:Log(clock);

      vel = {};                                -- stores velocity of target at 'snapshot'
      accel = {};                              -- stores acceleration of target at 'snapshot'
      jrk = {};                                -- stores jerk of target at 'snapshot'
	  snp = {};                                -- stores snap of target at 'snapshot'

-- Perform Backwards Finite Difference [O(h^2)]
                                               -- Weighted Summation of Last 3 positions (x)
      velx = TGTD[snap][2].Position[1] * 3;
      val = TGTD[snap-1][2].Position[1] * 4;
      velx = velx - val;
      velx = velx + TGTD[snap-2][2].Position[1];

         -- Divide by stepsize * 2
      step = freq/20;                          -- step size *2 [sec]
      velx = velx/step;                        -- should now be good approx. of z velocity


      vel[1] = velx;                           -- save velocity vector into velocity array


                                               -- Weighted Summation of Last 3 positions (y)
      vely = TGTD[snap][2].Position[2] * 3;
      val = TGTD[snap-1][2].Position[2] * 4;
      vely = vely - val;
      vely = vely + TGTD[snap-2][2].Position[2];

         -- Divide by stepsize * 2

      vely = vely/step;                        -- should now be good approx. of z velocity

      vel[2] = vely;                           -- save y velocity into velocity vector


                                               -- Weighted Summation of Last 3 positions (z)
      velz = TGTD[snap][2].Position[3] * 3;
      val = TGTD[snap-1][2].Position[3] * 4;
      velz = velz - val;
      velz = velz + TGTD[snap-2][2].Position[3];

         -- Divide by stepsize * 2
      velz = velz/step;                        -- should now be good approx. of z velocity

      vel[3] = velz;                           -- save z velocity into velocity vector


         -- MOD set velocity relative to vehicle
      vcor = I:GetVelocityVector();
      vel[1] = vel[1] - vcor[1];
      vel[2] = vel[2] - vcor[2];
      vel[3] = vel[3] - vcor[3];
      TGTS[1] = vel;                           -- save velocity vector into state array

-- Perform Backwards Finite Difference [O(h^2)] for Acceleration
                                               -- Weighted Summation of Last 4 Positions (x)
      accelx = TGTD[snap][2].Position[1] * 2;
      val = TGTD[snap-1][2].Position[1] * 5;
      accelx = accelx - val;
      val = TGTD[snap-2][2].Position[1] * 4;
      accelx = accelx + val;
      accelx = accelx - TGTD[snap-3][2].Position[1];

         -- Divide by the step size ^2
      step = step*2;                           -- Step size (h) [sec]
      steps = step*step;                       -- (h^2)
      accelx = accelx/steps;                   -- should be good approx. of x acceleration

      accel[1] = accelx;                       -- save x accel. into accel. vector


                                               -- Weighted Summation of Last 4 Positions (y)
      accely = TGTD[snap][2].Position[2] * 2;
      val = TGTD[snap-1][2].Position[2] * 5;
      accely = accely - val;
      val = TGTD[snap-2][2].Position[2] * 4;
      accely = accely + val;
      accely = accely - TGTD[snap-3][2].Position[2];

         -- Divide by the step size ^2
      accely = accely/steps;                   -- should be good approx. of y acceleration

      accel[2] = accely;                       -- save y accel. into accel. vector


                                               -- Weighted Summation of Last 4 Positions (z)
      accelz = TGTD[snap][2].Position[3] * 2;
      val = TGTD[snap-1][2].Position[3] * 5;
      accelz = accelz - val;
      val = TGTD[snap-2][2].Position[3] * 4;
      accelz = accelz + val;
      accelz = accelz - TGTD[snap-3][2].Position[3];

         -- Divide by the step size ^2
      accelz = accelz/steps;                     -- should be good approx. of z acceleration

      accel[3] = accelz;                       -- save z accel. into accel. vector

      TGTS[2] = accel;                         -- save acceleration vector into state array

-- Perform Backwards Finite Difference [O(h^2)] For Jerk

                                               -- Weighted Summation of Last 5 Positions (x)
      jrkx = TGTD[snap][2].Position[1] * 5;
      val = TGTD[snap-1][2].Position[1] * 18;
      jrkx = jrkx - val;
      val = TGTD[snap-2][2].Position[1] * 24;
      jrkx = jrkx + val;
	  val = TGTD[snap-3][2].Position[1] * 14;
      jrkx = jrkx - val;
	  val = TGTD[snap-4][2].Position[1] * 3;
      jrkx = jrkx + val;

         -- Divide by the step size ^3
      stepc = steps*step*2;                       -- (2h^3)
      jrkx = jrkx/stepc;                   -- should be good approx. of x jerk

      jrk[1] = jrkx;                       -- save x jerk into jerk vector


                                               -- Weighted Summation of Last 5 Positions (y)
      jrky = TGTD[snap][2].Position[2] * 5;
      val = TGTD[snap-1][2].Position[2] * 18;
      jrky = jrky - val;
      val = TGTD[snap-2][2].Position[2] * 24;
      jrky = jrky + val;
	  val = TGTD[snap-3][2].Position[2] * 14;
      jrky = jrky - val;
	  val = TGTD[snap-4][2].Position[2] * 3;
      jrky = jrky + val;

         -- Divide by the step size ^3

      jrky = jrky/stepc;                   -- should be good approx. of y jerk

      jrk[2] = jrky;                       -- save y jerk into jerk vector


                                               -- Weighted Summation of Last 5 Positions (z)
      jrkz = TGTD[snap][2].Position[3] * 5;
      val = TGTD[snap-1][2].Position[3] * 18;
      jrkz = jrkz - val;
      val = TGTD[snap-2][2].Position[3] * 24;
      jrkz = jrkz + val;
	  val = TGTD[snap-3][2].Position[3] * 14;
      jrkz = jrkz - val;
	  val = TGTD[snap-4][2].Position[3] * 3;
      jrkz = jrkz + val;

         -- Divide by the step size ^3

      jrkz = jrkz/stepc;                   -- should be good approx. of z jerk

      jrk[3] = jrkz;                       -- save z jerk into jerk vector

      TGTS[3] = jrk;                       -- save jerk vector into state array

-- Perform Backwards Finite Difference [O(h^2)] For 4th order term (snap)
	  stepd = steps*steps;                       -- (h^4)

                                         -- Weighted Summation of Last 6 Positions (x)
      snpx = TGTD[snap][2].Position[1] * 3;
      val = TGTD[snap-1][2].Position[1] * 14;
      snpx = snpx - val;
      val = TGTD[snap-2][2].Position[1] * 26;
      snpx = snpx + val;
	  val = TGTD[snap-3][2].Position[1] * 24;
      snpx = snpx - val;
	  val = TGTD[snap-4][2].Position[1] * 11;
      snpx = snpx + val;
	  val = TGTD[snap-5][2].Position[1] * 2;
      snpx = snpx - val;
	  
	      -- Divide by the step size ^4

	  snpx = snpx / stepd;
	  
	  snp[1] = snpx;
	  
	                                     -- Weighted Summation of Last 6 Positions (y)
      snpy = TGTD[snap][2].Position[2] * 3;
      val = TGTD[snap-1][2].Position[2] * 14;
      snpy = snpy - val;
      val = TGTD[snap-2][2].Position[2] * 26;
      snpy = snpy + val;
	  val = TGTD[snap-3][2].Position[2] * 24;
      snpy = snpy - val;
	  val = TGTD[snap-4][2].Position[2] * 11;
      snpy = snpy + val;
	  val = TGTD[snap-5][2].Position[2] * 2;
      snpy = snpy - val;
	  
	      -- Divide by the step size ^4

	  snpy = snpy / stepd;
	  
	  snp[2] = snpy;
	  
	                                     -- Weighted Summation of Last 6 Positions (z)
      snpy = TGTD[snap][2].Position[3] * 3;
      val = TGTD[snap-1][2].Position[3] * 14;
      snpy = snpy - val;
      val = TGTD[snap-2][2].Position[3] * 26;
      snpy = snpy + val;
	  val = TGTD[snap-3][2].Position[3] * 24;
      snpy = snpy - val;
	  val = TGTD[snap-4][2].Position[3] * 11;
      snpy = snpy + val;
	  val = TGTD[snap-5][2].Position[3] * 2;
      snpy = snpy - val;
	  
	      -- Divide by the step size ^4

	  snpy = snpy / stepd;
	  
	  snp[3] = snpy;
	  
	  TGTS[4] = snp;
   end
end

   -- Find Target Heading?
   -- Place Aim Point

if clock > 8*freq then                 -- I have no idea why it needs to wait this long

   if TGTD[snap][1].Valid == true then  
      wep = I:GetWeaponCount();             -- Code Breaks if this is global for some reason
      tim = counter/40;

      for idx = 0,wep-1,1 do

      
         info = I:GetWeaponInfo(idx);
         istur = false;

         if info.WeaponType == 4 then
            SubI = I:GetWeaponBlockInfo(idx).SubConstructIdentifier;
            info = I:GetWeaponInfoOnSubConstruct(SubI,0);
            istur = true;
         end

         loc = {};
         loc[1] = info.GlobalPosition[1];
    -- Get turret position
         loc[2] = info.GlobalPosition[2];
    -- Get turret position
         loc[3] = info.GlobalPosition[3];
    -- Get turret position

         if info.WeaponType == 2 and info.WeaponSlotMask == 1 then
   -- What follows is quadratic target prediction for laser
            dispx = TGTS[1][1] * tim;
                   -- Linear Correction Term
            dispy = TGTS[1][2] * tim;
            dispz = TGTS[1][3] * tim;

            varx = TGTS[2][1] * tim * tim * 0.5;        -- Quadradic Correction Term
            vary = TGTS[2][2] * tim * tim * 0.5;
        -- This is plenty for lasers
            varz = TGTS[2][3] * tim * tim * 0.5;

            dispx = dispx + varx;                       -- Sum Together
            dispx = dispx + varx;
            dispx = dispx + varx;

      -- Aim Turret
            dir1 = TGTC[snap][1] - loc[1] + dispx;
            dir2 = TGTC[snap][2] - loc[2] + dispy;
            dir3 = TGTC[snap][3] - loc[3] + dispz;

      --Fire Turret
            I:AimWeaponInDirection(idx,dir1,dir2,dir3,0);
            I:FireWeapon(idx,0);
         end

         if info.WeaponType == 0 and info.WeaponSlotMask == 1 then
            mVel = info.Speed;                            -- Get Speed of Projectile

            CXRng = TGTC[snap][1] - loc[1];               -- Get current dX
            CYRng = TGTC[snap][2] - loc[2];               -- Get current dY
            CZRng = TGTC[snap][3] - loc[3];               -- Get current dZ

            CXRngs = CXRng * CXRng;                       -- Find Lateral Range
            CZRngs = CZRng * CZRng;
            CLRng = math.sqrt(CXRngs + CZRngs);

         -- Find Ideal Launch Elevation for current position
            mVels = mVel * mVel;                          -- Find V^2
            mVelf = mVels * mVels;                        -- Find V^4

            var1 = G*CLRng;
            var2 = var1*CLRng;
            var3 = 2*CYRng*mVels;
            var4 = var2+var3;
            var5 = var4*G;
            var6 = math.sqrt(mVelf-var5);
            var7 = mVels - var6;
            theta1 = math.atan(var7/var1);

         -- Determine temporal error
            Lvel = math.cos(theta1) * mVel;
            T1 = CLRng / Lvel;
            T1E = tim+T1;
			
		 -- Get Multiples of T1 to make math quick
		    T12 = T1*T1;
		    T13 = T12*T1;
		    T14 = T13*T1;

         -- Find Position of target in previous raw travel time
            FXdispV = TGTS[1][1]*T1;                      -- Linear Corrector
            FYdispV = TGTS[1][2]*T1;
            FZdispV = TGTS[1][3]*T1;

            FXdispA = TGTS[2][1]*T12*0.5;                 -- Quadradic Corrector
            FYdispA = TGTS[2][2]*T12*0.5;
            FZdispA = TGTS[2][3]*T12*0.5;

            frak = 1/6;
            FXdispJ = TGTS[3][1]*T13*frak;                -- Cubic Corrector
            FYdispJ = TGTS[3][2]*T13*frak;
            FZdispJ = TGTS[3][3]*T13*frak;
			
			frak2 = 1/24;                                 -- Quartic Corrector
			FXdispS = TGTS[4][1]*T14*frak2;
			FYdispS = TGTS[4][2]*T14*frak2;
			FZdispS = TGTS[4][3]*T14*frak2;

            FXRng = CXRng + FXdispV + FXdispA + FXdispJ + FXdispS;  -- Future dX
            FYRng = CYRng + FYdispV + FYdispA + FYdispJ + FYdispS;  -- Future dY
            FZRng = CZRng + FZdispV + FZdispA + FZdispJ + FZdispS;  -- Future dZ

            FXRngs = FXRng * FXRng;                       -- Find Lateral Range
            FZRngs = FZRng * FZRng;
            FLRng = math.sqrt(FXRngs + FZRngs);

         -- Find Ideal Launch Elevation for future position

            var1 = G*FLRng;
            var2 = var1*FLRng;
            var3 = 2*FYRng*mVels;
            var4 = var2+var3;
            var5 = var4*G;
            var6 = math.sqrt(mVelf-var5);
            var7 = mVels - var6;
            theta2 = math.atan(var7/var1);

         -- Determine temporal error
            Lvel = math.cos(theta2) * mVel;
            T2 = FLRng / Lvel;
            T2E = T2 - T1 + tim;

         -- Linearly interpolate error
            TEsN = T2-T1;
            TEsD = T2E-T1E;
            TEs = TEsN/TEsD;                              -- M in y=mx+b
            MX = TEs*T1E;                                 -- MX in y=mx+b
            TT = T1-MX;                                   -- b=y-mx
			
		 -- Get Multiples of TT to make math faster
		    TT2 = TT*TT;
			TT3 = TT2*TT;
			TT4 = TT3*TT;

         -- Find target aimpoint
            TXdispV = TGTS[1][1]*TT;                      -- Linear Corrector
            TYdispV = TGTS[1][2]*TT;
            TZdispV = TGTS[1][3]*TT;

            TXdispA = TGTS[2][1]*TT2*0.5;                 -- Quadradic Corrector
            TYdispA = TGTS[2][2]*TT2*0.5;
            TZdispA = TGTS[2][3]*TT2*0.5;

            TXdispJ = TGTS[3][1]*TT3*frak;                -- Cubic Corrector
            TYdispJ = TGTS[3][2]*TT3*frak;
            TZdispJ = TGTS[3][3]*TT3*frak;
			
			TXdispS = TGTS[4][1]*TT4*frak2;               -- Quartic Corrector
			TYdispS = TGTS[4][2]*TT4*frak2;
			TZdispS = TGTS[4][3]*TT4*frak2;

            TXRng = CXRng + TXdispV + TXdispA + TXdispJ + TXdispS;  -- Target dX
            TYRng = CYRng + TYdispV + TYdispA + TYdispJ + TYdispS;  -- Target dY
            TZRng = CZRng + TZdispV + TZdispA + TZdispJ + TZdispS;  -- Target dZ

         -- Find target elevation
            TXRngs = TXRng * TXRng;                       -- Find Lateral Range
            TZRngs = TZRng * TZRng;
            TLRng = math.sqrt(TXRngs + TZRngs);

            var1 = G*TLRng;
            var2 = var1*TLRng;
            var3 = 2*TYRng*mVels;
            var4 = var2+var3;
            var5 = var4*G;
            var6 = math.sqrt(mVelf-var5);
            var7 = mVels - var6;
            TanT = var7/var1;

         -- Aim Turret in this direction
            Ydir = TanT * TLRng;
            I:AimWeaponInDirection(idx,TXRng,Ydir,TZRng,0);
         -- Fire Turret
            I:FireWeapon(idx,0);
         end
      end

   end

end
end