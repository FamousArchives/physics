/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Owner: david@famo.us
 * @license MPL 2.0
 * @copyright Famous Industries, Inc. 2014
 */

define(function(require, exports, module) {
    var Constraint = require('./Constraint');
    var Vector = require('famous/math/Vector');

    /**
     *  Allows for two circular bodies to collide and bounce off each other.
     *
     *  @class Collision
     *  @constructor
     *  @extends Constraint
     *  @param {Options} [options] An object of configurable options.
     *  @param {Number} [options.restitution] The energy ratio lost in a collision (0 = stick, 1 = elastic) Range : [0, 1]
     *  @param {Number} [options.drift] Baumgarte stabilization parameter. Makes constraints "loosely" (0) or "tightly" (1) enforced. Range : [0, 1]
     *  @param {Number} [options.slop] Amount of penetration in pixels to ignore before collision event triggers
     *
     */
    function Collision(options) {
        this.options = Object.create(Collision.DEFAULT_OPTIONS);
        if (options) this.setOptions(options);

        //registers
        this.normal   = new Vector();
        this.pDiff    = new Vector();
        this.vDiff    = new Vector();
        this.impulse1 = new Vector();
        this.impulse2 = new Vector();
        this.torque1  = new Vector();
        this.torque2  = new Vector();

        Constraint.call(this);
    }

    Collision.prototype = Object.create(Constraint.prototype);
    Collision.prototype.constructor = Collision;

    Collision.DEFAULT_OPTIONS = {
        restitution : 0.5,
        drift : 0.5,
        slop : 0
    };

    function _normalVelocity(particle1, particle2) {
        return particle1.velocity.dot(particle2.velocity);
    }

    /*
     * Setter for options.
     *
     * @method setOptions
     * @param options {Objects}
     */
    Collision.prototype.setOptions = function setOptions(options) {
        for (var key in options) this.options[key] = options[key];
    };

    /**
     * Adds an impulse to a physics body's velocity due to the constraint
     *
     * @method applyConstraint
     * @param targets {Array.Body}  Array of bodies to apply the constraint to
     * @param source {Body}         The source of the constraint
     * @param dt {Number}           Delta time
     */
    Collision.prototype.applyConstraint = function applyConstraint(targets, source, dt) {
        if (source === undefined) return;

        var v1 = source.velocity;
        var p1 = source.position;
        var w1 = source.inverseMass;

	if (source instanceof Circle) {
                var r1 = source.radius;
	}
	else if (source instanceof Ellipse) {
	}
	else if (source instanceof Rectangle) {
            var o1 = source.orientation;
	}

        var options = this.options;
        var drift = options.drift;
        var slop = -options.slop;
        var restitution = options.restitution;

        var n     = this.normal;
        var pDiff = this.pDiff;
        var vDiff = this.vDiff;
        var impulse1 = this.impulse1;
        var impulse2 = this.impulse2;
        var torque1 = this.torque1;
        var torque2 = this.torque2;

        for (var i = 0; i < targets.length; i++) {
            var target = targets[i];

            if (target === source) continue;

            var v2 = target.velocity;
            var p2 = target.position;
            var w2 = target.inverseMass;

            if (target instanceof Rectangle) {
                var o2 = target.orientation;
            }

            pDiff.set(p2.sub(p1));
            vDiff.set(v2.sub(v1));

            var dist    = pDiff.norm();
            var effMass = 1/(w1 + w2);
            var gamma   = 0;

            if (target instanceof Circle) {
                var r1 = target.radius;
            }
            else if (target instanceof Ellispe) {
            }
            else if (target instanceof Square) {
                var theta = Math.atan(pDiff.y/pDiff.x);
                if (source instanceof Square) {
		    var r1 = source.size[0]/2 * Math.cos(theta);
                }
                var r2 = target.size[0]/2 * Math.cos(theta);
            }
            else if (target instanceof Rectangle) {
            }

            var overlap = dist - (r1 + r2);

            if (overlap < 0) {

                if ((source instanceof Circle) && (target instanceof Circle)) {
                    n.set(pDiff.normalize());
                }
                if ((source instanceof Square) && (target instanceof Square)) {
                    if ((theta < Math.PI/4) && theta > -1*Math.PI/4) {
                        n.set(1);
                    }
                    if ((theta > Math.PI/4) && theta < -1*Math.PI/4) {
                        n.set([0,1,0]);
                    }
                }

                if (this._eventOutput) {
                    var collisionData = {
                        target  : target,
                        source  : source,
                        overlap : overlap,
                        normal  : n
                    };

                    this._eventOutput.emit('preCollision', collisionData);
                    this._eventOutput.emit('collision', collisionData);
                }

		//if ((source instanceof Circle) && (target instanceof Circle)) {
                    var lambda = (overlap <= slop)
                        ? ((1 + restitution) * n.dot(vDiff) + drift/dt * (overlap - slop)) / (gamma + dt/effMass)
                        : ((1 + restitution) * n.dot(vDiff)) / (gamma + dt/effMass);
                //}
                //if ((source instanceof Square) && (target instanceof Square)) {
                //    var vtheta = Math.atan(vDiff.y/vDiff.x);
                //    var lambda = (overlap <= slop)
                //        ? ((1 + restitution) * n.dot(vDiff) + drift/dt * (overlap - slop)) / (gamma + dt/effMass)
                //        : ((1 + restitution) * n.dot(vDiff)) / (gamma + dt/effMass);
                //}

                n.mult(dt*lambda).put(impulse1);
                impulse1.mult(-1).put(impulse2);

                source.applyImpulse(impulse1);
                target.applyImpulse(impulse2);

		// Calculate the torque for off center collisions of Rectangular bodies.
                // Normal orientation and same size of source and target.
                else if (target instanceof Rectangle) {
			var fdist = pDiff.y.mult(0.5);
                    	var torque1 = (overlap <= slop)
                        	? (vDiff.cross(fdist).mult(1 + restitution) + drift/dt * (overlap - slop)) / (gamma + dt/effMass)
                        	: vDiff.cross(fdist).mult((1 + restitution)/(gamma + dt/effMass));
			//vDiff.cross(fdist).put(torque1);
                        torque1.mult(-1).put(torque2);
			source.applyTorque(torque1);
			target.applyTorque(torque2);
                }


                //source.setPosition(p1.add(n.mult(overlap/2)));
                //target.setPosition(p2.sub(n.mult(overlap/2)));

                if (this._eventOutput) this._eventOutput.emit('postCollision', collisionData);

            }
        }
    };

    module.exports = Collision;
});
