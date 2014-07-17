/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Owner: david@famo.us
 * @license MPL 2.0
 * @copyright Famous Industries, Inc. 2014
 */

define(function(require, exports, module) {
    var Body = require('./Rectangle');
    var Matrix = require('famous/math/Matrix');

    /**
     * Implements a square geometry for an Body with
     * size = length.
     *
     * @class Square
     * @extends Rectangle
     * @constructor
     */
    function Square(options) {
        options = options || {};
        this.size = options.size || [0,0];
        Rectangle.call(this, options);
    }

    Square.prototype = Object.create(Rectangle.prototype);
    Square.prototype.constructor = Square;

    /**
     * Basic setter for size.
     * @method setSize
     * @param length
     */
    Square.prototype.setSize = function setSize(length) {
	var size = [length, length];
        this.size = size;
        this.setMomentsOfInertia();
    };

    module.exports = Square;

});
