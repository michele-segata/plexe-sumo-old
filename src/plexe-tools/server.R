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

library(shiny)
library(XML)
source('shiny-engine.R')

add.idx <- function(txt, index) {
	return (paste(txt, index, sep=''))
}

get.models <- function() {
	xmld <- xmlParse('./vehicles.xml')
	xd <- xmlToList(xmld)

	models <- c()
	for (i in 1:length(xd))
		models <- c(models, xd[[i]]$.attrs[['id']])
	return (models)
}
get.car <- function(car) {
	xmld <- xmlParse('./vehicles.xml')
	xd <- xmlToList(xmld)

	for (i in 1:length(xd))
		if (xd[[i]]$.attrs[['id']] == car)
			return(i)
}
get.gear <- function(gears, gear) {
	if (gear >= length(gears))
		return (1)
	for (i in 1:length(gears))
		if (gears[[i]][['n']] == gear)
			return(gears[[i]][['ratio']])
}
get.coeffs <- function(pw) {
	coeffs <- c()
	for (i in 1:length(pw))
		coeffs <- c(coeffs, as.numeric(pw[[paste('x', i-1, sep='')]]))
	return (coeffs)
}

shinyServer(function(input, output, session) {

	coeffs <- c()
	observe({
		car <- input$car
		if (is.null(car)) {
			models <- get.models()
			car <- models[1]
		}

		xmld <- xmlParse('./vehicles.xml')
		xd <- xmlToList(xmld)
		#user changed selected vehicle. we need to load data from xml
		v.index <- get.car(car)
		v <- xd[[v.index]]
		coeffs <<- get.coeffs(v$engine$power)
		#number of gears
		def.n.gears         <- length(v$gears) - 1
		#gears ratio
		def.gears <- c()
		for (i in 1:(def.n.gears))
			def.gears       <- c(def.gears, get.gear(v$gears, i))
		def.differential    <- v$gears$differential[['ratio']]
		def.minRpm          <- v$engine$.attrs[['minRpm']]
		def.maxRpm          <- v$engine$.attrs[['maxRpm']]
		def.eta             <- v$engine$.attrs[['efficiency']]
		def.mass            <- v$mass[['mass']]
		def.massCoefficient <- v$mass[['massFactor']]
		def.cAir            <- v$drag[['cAir']]
		def.section         <- v$drag[['section']]
		def.wheeld          <- v$wheels[['diameter']]
		def.wheelf          <- v$wheels[['friction']]

		for (i in 1:(def.n.gears))
			updateSliderInput(session, inputId = paste('ratio', i, sep=''), value=def.gears[i])
		updateSliderInput(session, inputId = "ngears"         , value=def.n.gears)
		updateSliderInput(session, inputId = "differential"   , value=def.differential)
		updateSliderInput(session, inputId = "minRpm"         , value=def.minRpm)
		updateSliderInput(session, inputId = "maxRpm"         , value=def.maxRpm)
		updateSliderInput(session, inputId = "eta"            , value=def.eta)
		updateSliderInput(session, inputId = "mass"           , value=def.mass)
		updateSliderInput(session, inputId = "massCoefficient", value=def.massCoefficient)
		updateSliderInput(session, inputId = "cAir"           , value=def.cAir)
		updateSliderInput(session, inputId = "section"        , value=def.section)
		updateSliderInput(session, inputId = "wheeld"         , value=def.wheeld)
		updateSliderInput(session, inputId = "wheelf"         , value=def.wheelf)
	})


	output$main_plot <- renderPlot({

	n.gears <- input$ngears

	gear.ratios <- c()

	for (i in c(1:n.gears)) {
		#horizontal polarizations for ground, close wall, and far wall
		gear.ratios <- c(gear.ratios, input[[paste('ratio', i, sep='')]])
	}

	min.speed <- rpm.to.v.mps(rpm=input$minRpm, d.wheel.m=input$wheeld, i.d=input$differential, i.g=input$ratio1)

	plot.thrust(
		accel=T,
		xlim=c(input$xmin,input$xmax),
		ylim=c(input$ymin,input$ymax),
		differential=input$differential,
		speed.kmph=seq(input$xmin,input$xmax),
		mass.kg=input$mass,
		mass.coefficient=input$massCoefficient,
		eta=input$eta,
		max.rpm=input$maxRpm,
		min.speed=min.speed,
		min.power.hp=engine.model.rpm.to.hp(rpm=input$minRpm),
		d.wheel.m=input$wheeld,
		cAir=input$cAir,
		a.m2=input$section,
		gear.ratios=gear.ratios,
		coeffs=coeffs,
		wheelFriction=input$wheelf
	)

  })
})
