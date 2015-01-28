/* 
 * Copyright (c) 2007 Karsten Schmidt
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 * 
 * http://creativecommons.org/licenses/LGPL/2.1/
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

package toxi.geom;

import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.XmlElement;

/**
 * A simple 3D ray datatype
 */
@XmlAccessorType(XmlAccessType.FIELD)
public class Ray3D extends Vec3D {

	@XmlElement(required = true)
	public Vec3D dir;

	public Ray3D() {
		super();
		dir = Vec3D.Y_AXIS.copy();
	}

	public Ray3D(float x, float y, float z, Vec3D d) {
		super(x, y, z);
		dir = d.getNormalized();
	}

	public Ray3D(Vec3D o, Vec3D d) {
		super(o);
		dir = d.getNormalized();
	}

	public Vec3D getDirection() {
		return dir.copy();
	}

	public Vec3D getPointAtDistance(float dist) {
		return add(dir.scale(dist));
	}

	public String toString() {
		return "origin: " + super.toString() + " dir: " + dir;
	}
}
