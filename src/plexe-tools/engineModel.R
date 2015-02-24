#
# Copright (c) 2015 Michele Segata <segata@ccs-labs.org>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see http://www.gnu.org/licenses/.
#

rpm.to.hp <- data.frame(
	rpm=c(1521, 2004, 2505, 3012, 3504, 4011, 4209, 4517, 5007, 5516, 6013, 6222, 6518, 6791),
	hp=c(17.3, 24.9, 32.9, 39.7, 47.8, 56.6, 60.5, 63.0, 69.0, 78.8, 83.2, 83.2, 82.3, 79.4)*1.34
)

min.rpm <- 500
max.rpm <- 7000

if (!interactive()) {
	#we are running from command line
	args <- commandArgs(trailingOnly=T)
	if (length(args) != 3) {
		cat("Usage: Rscript engineModel.R <engine mapping (csv file)> <min rpm> <max rpm>\n")
		cat("The csv file should contain two columns named 'rpm' (for engine rotation speed) and either 'hp' or 'kw' (for engine power in horsepower or kilowatts respectively\n")
		stop()
	}
	rpm.to.hp <- read.csv(args[1])
	#if power is expressed in kw, convert it to hp
	if ('kw' %in% names(rpm.to.hp)) {
		rpm.to.hp$hp <- rpm.to.hp$kw * 1.34
	}
	min.rpm <- as.numeric(args[2])
	max.rpm <- as.numeric(args[3])
}

if (!interactive())
	pdf('engineModel.pdf', width=16, height=9)

par(mfrow=c(3, 3))

torque.Nm <- function(rpm, power.hp) {
	power.w <- power.hp * 745.699872
	radps <- rpm / 60.0 * 2 * pi
	return (power.w / radps)
}

for (i in 1:9) {
	plot.new()
	plot.window(xlim=c(0, max.rpm), ylim=c(0, round(max(rpm.to.hp$hp))+10))
	m <- lm(hp ~ poly(rpm, i, raw=T), data=rpm.to.hp)
	x <- seq(0, max.rpm)
	y <- predict(m, newdata=data.frame(rpm=x))
	lwd <- 2
	lines(x, y, col='red', lwd=lwd)
	abline(v=min.rpm, col='blue', lwd=2)
	abline(v=max.rpm, col='darkgreen', lwd=2)
	abline(h=predict(m, newdata=data.frame(rpm=min.rpm)), col='blue', lwd=2)
	abline(h=predict(m, newdata=data.frame(rpm=max.rpm)), col='darkgreen', lwd=2)
	points(rpm.to.hp$rpm, rpm.to.hp$hp, type='p', pch=21, bg='black')
	cex=0.7
	title(main=paste('degree =', i))
	mtext('rpm', side=1, line=2, cex=cex)
	mtext('power (hp)', side=2, line=2, cex=cex)
	axis(2)

	par(new=T)

	min.meas.rpm <- min(rpm.to.hp$rpm)
	max.meas.rpm <- max(rpm.to.hp$rpm)
	rpm.range <- min.meas.rpm:max.meas.rpm
	torque <- torque.Nm(rpm.range, predict(m, newdata=data.frame(rpm=rpm.range)))
	rmin <- min(torque)
	rmax <- max(torque)
	plot.window(xlim=c(0, max.rpm), ylim=c(round(rmin)-20, round(rmax)+20))
	lines(rpm.range, torque, col='darkorange', lwd=2)
	mtext("torque (Nm)", side=4, line=2, cex=cex)
	axis(4)

	axis(1)
	box()

	#print xml parameters for the engine model
	cat(paste('XML parameters for polynomial model of degree', i, '\n'))
	cat(paste('\t<engine efficiency="0.8" cylinders="4" type="poly" maxRpm="', max.rpm, '" minRpm="', min.rpm, '">\n', sep=''))
	cat("\t\t<power")
	for (x in 1:(i+1)) {
		cat(paste(' x', (x-1), '="', m$coefficients[x], '"', sep=''))
	}
	cat('/>\n\t</engine>\n\n')

}

if (!interactive())
	dev.off()

