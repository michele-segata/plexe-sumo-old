cl <- list()
cl[['base03']] <-    '#002b36'
cl[['base02']] <-    '#073642'
cl[['base01']] <-    '#586e75'
cl[['base00']] <-    '#657b83'
cl[['base0']] <-     '#839496'
cl[['base1']] <-     '#93a1a1'
cl[['base2']] <-     '#eee8d5'
cl[['base3']] <-     '#fdf6e3'
cl[['yellow']] <-    '#b58900'
cl[['orange']] <-    '#cb4b16'
cl[['red']] <-       '#dc322f'
cl[['magenta']] <-   '#d33682'
cl[['violet']] <-    '#6c71c4'
cl[['blue']] <-      '#268bd2'
cl[['cyan']] <-      '#2aa198'
cl[['green']] <-     '#859900'
lcolors <- c(cl$base01, cl$red, cl$orange, cl$yellow, cl$green, cl$cyan, cl$blue, cl$violet)

engine.model.gen <- function(rpm.to.hp) {
    model <- lm(hp~rpm, data=rpm.to.hp)
    return (function(rpm) {return(predict(model, newdata=data.frame(rpm=rpm)))})
}
rpm.to.hp <- data.frame(
    rpm=c(1521  , 2004  , 2505  , 3012  , 3504  , 4011  , 4209  , 4517  , 5007  , 5516) , #c(800 , 1200 , 1600 , 1800 , 2000 , 2200 , 2400) ,
    hp =c(23.18 , 33.36 , 44.08 , 53.19 , 64.05 , 75.84 , 81.07 , 84.42 , 92.46 , 105.59) #c(75 , 130    , 165  , 170  , 190  , 203  , 210)
)
rpm.to.hp <- data.frame(
	rpm=c(1521, 2004, 2505, 3012, 3504, 4011, 4209, 4517, 5007, 5516, 6013, 6222, 6518, 6791),
	hp=c(17.3, 24.9, 32.9, 39.7, 47.8, 56.6, 60.5, 63.0, 69.0, 78.8, 83.2, 83.2, 82.3, 79.4)*1.34
)
rpm.to.hp <- data.frame(
	rpm=c(1521, 2004, 2505, 3012, 3504, 4011, 4209, 4517, 5007, 5516, 6013),
	hp=c(17.3, 24.9, 32.9, 39.7, 47.8, 56.6, 60.5, 63.0, 69.0, 78.8, 83.2)*1.34
)

rpm.to.v.mps <- function(rpm, d.wheel.m=0.94, i.d=4.6, i.g=4.5) {
	#compute speed as function of rpm, so getting meters per minute
	v.mpm <- rpm * d.wheel.m * pi / (i.d * i.g)
	#return speed in meters per second
	return (v.mpm / 60.0)
}

v.mps.to.rpm <- function(v.mps, d.wheel.m=0.94, i.d=4.6, i.g=4.5) {
	#compute engine rps
	rps <- v.mps * i.d * i.g / (d.wheel.m * pi)
	#convert rps to rpm
	return (rps*60)
}

#hp is in kg * m^2 / s^3
v.mps.to.hp <- function(v.mps, rpm.to.hp.model, d.wheel.m=0.94, i.d=4.6, i.g=4.5) {
	rpm <- v.mps.to.rpm(v.mps, d.wheel.m, i.d, i.g)
	return (rpm.to.hp.model(rpm))
}

v.mps.to.thrust.n <- function(v.mps, rpm.to.hp.model, d.wheel.m=0.94, i.d=4.6, i.g=4.5, eta=0.8) {
	hp <- v.mps.to.hp(v.mps, rpm.to.hp.model, d.wheel.m, i.d, i.g)
	hp <- pmax(hp, 0)
	return (eta * hp / v.mps / 3.6 * 2650)
}

get.gear <- function(v.mps, ratios, max.rpm=4000, d.wheel.m=0.94, i.d=4.6) {
	gears <- c()
	for (i in 1:length(v.mps)) {
		for (gear in 1:length(ratios)) {
			rpm <- v.mps.to.rpm(v.mps[i], d.wheel.m, i.d, i.g=ratios[gear])
			if (rpm < max.rpm | gear == length(ratios)) {
				gears <- c(gears, gear)
				break
			}
		}
	}
	return(gears)
}

air.drag.n <- function(v.mps, c.air=0.3, al.m2=2.7, rho.kgpm3=1.2) {
	return(0.5*c.air*al.m2*rho.kgpm3*v.mps**2)
}

rolling.n <- function(v.mps, mass.kg, cr1=0.0136, cr2=5.18e-7) {
	return(mass.kg*9.81*(cr1+cr2*v.mps**2))
}

gravity.n <- function(mass.kg, slope) {
	return(mass.kg*9.81*sin(slope/180*pi))
}

Fl.n <- function(v.mps, mass.kg, slope = 0, c.air=0.3, al.m2=2.7, rho.kgpm3=1.2, cr1=0.0136, cr2=5.18e-7) {
	return (
	    air.drag.n(v.mps, c.air, al.m2, rho.kgpm3) +
	    rolling.n(v.mps, mass.kg, cr1, cr2) +
	    gravity.n(mass.kg, slope)
	)
}

#maximum acceleration taking into account tyre friction
max.friction.acceleration.n <- function(slope=0, friction.coeff=0.7, g.mps2=9.81) {
	return (friction.coeff * g.mps2 * cos(slope/180*pi))
}


plot.thrust <- function(accel=F, xlim, ylim, differential, speed.kmph, mass.kg, mass.coefficient, eta, max.rpm, min.speed, min.power.hp, d.wheel.m, cAir, a.m2, gear.ratios, coeffs, wheelFriction) {

	engine.model.rpm.to.hp <- function(rpm=1000) {
		s <- coeffs[1]
		for (i in 2:length(coeffs))
			s <- s + coeffs[i]*rpm**(i-1)
		return (s)
	}

	plot.new()
	plot.window(xlim=xlim, ylim=ylim, xaxs='i', yaxs='i')

	#compute thrust
	for (gear in 1:length(gear.ratios)) {
		to.plot <- data.frame(
			speed=speed.kmph,
			rpm=v.mps.to.rpm(speed.kmph/3.6, d.wheel.m=d.wheel.m, i.d=differential, i.g=gear.ratios[gear]),
			thrust=pmax(
			    0,
			    v.mps.to.thrust.n(
			        pmax(min.speed, speed.kmph/3.6),
			        engine.model.rpm.to.hp,
			        d.wheel.m=d.wheel.m,
			        i.d=differential,
			        i.g=gear.ratios[gear],
			        eta=eta
			    ) - Fl.n(speed.kmph/3.6, mass.kg, slope=0, c.air=cAir, al.m2=a.m2)),
			engine.thrust=v.mps.to.thrust.n(
			    pmax(min.speed, speed.kmph/3.6),
			    engine.model.rpm.to.hp,
			    d.wheel.m=d.wheel.m,
			    i.d=differential,
			    i.g=gear.ratios[gear],
			    eta=eta
			)
		)
		to.plot$max.accel <- pmin(to.plot$thrust / mass.coefficient / mass.kg, max.friction.acceleration.n(friction.coeff=wheelFriction))
		to.plot$max.engine.accel <- to.plot$engine.thrust / mass.coefficient / mass.kg
		if (gear != 1)
			to.plot <- subset(to.plot, rpm >= 1000 & rpm <= max.rpm)
		else
			to.plot <- subset(to.plot, rpm <= max.rpm)
		if (accel) {
			lines(to.plot$speed, to.plot$max.accel, col=lcolors[gear], lwd=2)
			lines(to.plot$speed, to.plot$max.engine.accel, col=lcolors[gear], lty=2, lwd=2)
		}
		else {
			lines(to.plot$speed, to.plot$thrust/1000, col=lcolors[gear])
			lines(to.plot$speed, to.plot$engine.thrust/1000, col=lcolors[gear], lty=2)
		}
	}

	axis(1)
	axis(2)
	title(xlab="speed (km/h)")
	if (accel)
		title(ylab="max acceleration (m/s/s)")
	else
		title(ylab="max force (kN)")
	box()


}
