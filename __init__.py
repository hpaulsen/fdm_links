import bpy
import bmesh
import math
from bpy.types import Operator
from bpy.props import FloatVectorProperty, EnumProperty, FloatProperty, IntProperty
from mathutils import Vector, Matrix, Euler

class HPaulsen_FDMJoint:
    def __init__(self,type,size,up_angle,down_angle,horizontal_angle,clearance,resolution):
        self.name = "3d Joint"
        self.type = type
        self.size = size # min size ~ 2.9mm
        self.resolution = resolution
        
        self.up_angle = up_angle
        self.down_angle = down_angle
        
        self.clearance = clearance # 0.3 seems generous, but is about the minimum recommended
        self.overlap = 0.3 #max(0.3,self.size/20) # each side
        self.print_angle = math.radians(45) # don't change without verifying areas where this is assumed
        self.a_step = 2*math.pi/resolution
        self.half_arm_width = max(0.6,0.3*self.size/2)
        self.wall_width = max(0.8,0.25*self.size/2)
        self.bridgeable_r = 1.5 # max safe bridgeable distance
        
        self.axle_min_r = max(0.4,self.size/6) # the radius of the endpoint of the "ball"
        self.axle_major_r = self.size/2 - self.clearance
        self.axle_x = math.sqrt(math.pow(self.axle_major_r,2)-math.pow(self.axle_min_r,2))
        self.axle_r = self.axle_min_r+(self.axle_x-self.half_arm_width)/math.tan(self.print_angle) #self.axle_min_r+self.overlap+self.clearance

        self.v_clearance = 0.6 # clearance between axle and roof of socket
        self.v_clearance_max = 2 # max clearance between axle and roof of socket
        self.bottom = 1.5*self.size # how low to go, rather arbitrary - just has to be long enough to be below z=0
        self.arm_length = self.size/2+self.wall_width+2*self.clearance#2*self.size
        self.separator_outer_r = 2*self.size
        self.separator_y = self.size/2+self.wall_width
        
        if self.axle_x-self.overlap < self.half_arm_width+self.clearance:
            raise RuntimeError("FDMJoint: Axle size is too small and/or Clearance is too large")
        
        self.bendable_angle = math.asin((self.axle_x-self.overlap-self.half_arm_width)/(self.size/2+self.clearance))
        if self.bendable_angle > horizontal_angle:
            self.bendable_angle = horizontal_angle
        elif self.bendable_angle < 0:
            self.bendable_angle = 0
            
        self.bottom_r = self.axle_x-self.overlap
        self.raise_z = 1.5*self.axle_r # somewhat arbitrary

    def loopx(self,verts,r,x):
        verts += [(x,r*math.sin(i*self.a_step),r*math.cos(i*self.a_step)) for i in range(self.resolution)]

    def loopz(self,verts,r,z,yscale=1.0):
        verts += [(r*math.sin(i*self.a_step),r*math.cos(i*self.a_step)*yscale,z) for i in range(self.resolution)]
    
    def loopy(self,verts,r,y):
        verts += [(r*math.sin(i*self.a_step),y,r*math.cos(i*self.a_step)) for i in range(self.resolution)]
        
    def bridge_last_loop(self,verts,faces,reverse_normals=False):
        vertslen = len(verts)
        lastloopstart = vertslen-self.resolution
        priorloopstart = lastloopstart-self.resolution
        if not reverse_normals:
            faces += [(priorloopstart+j+1,priorloopstart+j,lastloopstart+j,lastloopstart+j+1) for j in range(self.resolution-1)]
            faces += [(priorloopstart,lastloopstart-1,vertslen-1,lastloopstart)]
        else:
            faces += [(lastloopstart+j+1,lastloopstart+j,priorloopstart+j,priorloopstart+j+1) for j in range(self.resolution-1)]
            faces += [(lastloopstart,vertslen-1,lastloopstart-1,priorloopstart)]
            
    def bridge_to_point(self,verts,faces,reverse_normals=False):
        if reverse_normals:
            tip_vert = len(verts)-1
            faces += [(tip_vert,tip_vert-self.resolution+i+1,tip_vert-self.resolution+i) for i in range(self.resolution-1)]
            faces += [(tip_vert,tip_vert-self.resolution,tip_vert-1)]
        else:
            tip_vert = len(verts)-self.resolution-1
            faces += [(0,i,i+1) for i in range(1,self.resolution)]
            faces += [(0,self.resolution,1)]
            
    def create_obj(self,verts,faces):
        mesh_data = bpy.data.meshes.new(f"{self.name}_data")
        mesh_obj = bpy.data.objects.new(self.name,mesh_data)
        bpy.context.scene.collection.objects.link(mesh_obj)
        bm = bmesh.new()
        for coord in verts:
            bm.verts.new(coord)
        bm.verts.ensure_lookup_table()
        for indices in faces:
            bm.faces.new([bm.verts[index] for index in indices])
        bm.to_mesh(mesh_data)
        mesh_data.update()
        bm.free()
        return mesh_obj
    
    def add_socket(self):
        verts = []
        faces = []

        bottom = -self.axle_min_r-self.size/2
        verts += [(0,0,bottom)]
        self.loopz(verts,self.size/2,-self.axle_min_r)
        self.bridge_to_point(verts,faces)
        self.loopz(verts,self.size/2,self.axle_min_r)
        self.bridge_last_loop(verts,faces)
        
        top = self.axle_r+self.v_clearance
        topr = self.size/2-(top-self.axle_min_r)
        if topr > self.bridgeable_r:
            d = topr-self.bridgeable_r
            topr = self.bridgeable_r
            if d > self.v_clearance_max:
                top = self.axle_r+self.v_clearance_max
                topr = self.size/2-(top-self.axle_min_r)
            else:
                top += d
                topr = self.bridgeable_r
            
        self.loopz(verts,topr,top)
        self.bridge_last_loop(verts,faces)
        faces += [(len(verts)-1-i for i in range(self.resolution))]
            
        return self.create_obj(verts,faces)
        
    def add_axle(self):
        verts = []
        faces = []
        
        straight_x = self.axle_x-(self.axle_r-self.axle_min_r)
        straight_x = max(straight_x,self.half_arm_width+0.0001)
        
        self.loopx(verts,self.axle_min_r,-self.axle_x)
        faces += [range(self.resolution)]
        self.loopx(verts,self.axle_r,-straight_x)
        self.bridge_last_loop(verts,faces)
        self.loopx(verts,self.axle_r,-self.half_arm_width)
        self.bridge_last_loop(verts,faces)
        l = len(verts)
        for i in range(l-self.resolution,l):
            if verts[i][2] < 0:
                verts[i] = (verts[i][0],verts[i][1],verts[i][2]-verts[i][0]-straight_x)
        self.loopx(verts,self.axle_r,self.half_arm_width)
        self.bridge_last_loop(verts,faces)
        l = len(verts)
        for i in range(l-self.resolution,l):
            if verts[i][2] < 0:
                verts[i] = (verts[i][0],verts[i][1],verts[i][2]+verts[i][0]-straight_x)
        self.loopx(verts,self.axle_r,straight_x)
        self.bridge_last_loop(verts,faces)
        self.loopx(verts,self.axle_min_r,self.axle_x)
        self.bridge_last_loop(verts,faces)
        faces += [(len(verts)-1-i for i in range(self.resolution))]
        
        return self.create_obj(verts,faces)
        
    def add_arm(self):
        verts = []
        faces = []
        
        x = self.half_arm_width
        
        verts += [(-x,0,self.axle_min_r),(x,0,self.axle_min_r),(-x,self.arm_length,self.axle_min_r),(x,self.arm_length,self.axle_min_r)]
        faces += [(0,1,3,2)]
        
        arm_start_z = self.axle_r*math.cos(self.print_angle)+self.axle_x-self.half_arm_width-(self.axle_r-self.axle_min_r)
        arm_start_y = self.axle_r*math.sin(self.print_angle)
        
        verts += [(-x,-arm_start_y,-arm_start_z),(x,-arm_start_y,-arm_start_z)]
        faces += [(4,5,1,0)]
        
        d = self.arm_length+arm_start_y
        verts += [(-x,-arm_start_y+d,-arm_start_z-d),(x,-arm_start_y+d,-arm_start_z-d)]
        faces += [(6,7,5,4),(2,3,7,6)]
        
        faces += [(0,2,6,4),(5,7,3,1)]
        
        return self.create_obj(verts,faces)

    def add_arm_space(self,depth=0):
        verts = []
        faces = []
        
        r = self.axle_major_r+self.clearance
        y = -r*math.cos(self.down_angle-self.print_angle)
        # note: the following may need to remove the last term if popping-out through the bottom occurs.
        x = self.half_arm_width+self.clearance-y*math.sin(self.bendable_angle)
        y = y*math.cos(self.bendable_angle)
        z = r*math.sin(self.down_angle-self.print_angle)
        verts += [(-x,y,z),(x,y,z)]
        h = self.bottom+z
        y = y-h*math.tan(self.down_angle-self.print_angle)
        x = self.half_arm_width+self.clearance+abs(y*math.sin(self.bendable_angle))
        verts += [(-x,y,-self.bottom),(x,y,-self.bottom)]
        faces += [(2,3,1,0)]
        
        if y < 0:
            x = self.half_arm_width+self.clearance
            y = 0
            z = -self.bottom
            verts += [(-x,y,z),(x,y,z)]
            faces += [(0,4,2),(1,3,5),(3,2,4,5)]
        
        x = self.half_arm_width+self.clearance
        y = 0
        z = self.axle_min_r
        verts += [(-x,y,z),(x,y,z)]
        l = len(verts)-1
        faces += [(0,1,l,l-1),(0,l-1,l-3),(1,l-2,l)]
        
        if 0 == depth:
            y = self.arm_length-self.clearance
        else:
            y = depth
        x = y*math.tan(self.bendable_angle)+self.half_arm_width
        x = max(x,self.half_arm_width+self.clearance)
        z = -self.bottom
        z_far_top = self.axle_min_r+y*math.tan(self.up_angle)
        verts += [(-x,y,z_far_top),(x,y,z_far_top),(-x,y,z),(x,y,z)]
        faces += [(l-1,l+1,l+3,l-3),(l-2,l+4,l+2,l),(l-3,l+3,l+4,l-2)]
        
        verts += [(0,0,self.axle_min_r+(self.clearance+self.half_arm_width)*math.tan(self.print_angle)),(0,y,z_far_top+x*math.tan(self.print_angle))]
        faces += [(l-1,l,l+5),(l+1,l-1,l+5,l+6),(l,l+2,l+6,l+5),(l+1,l+6,l+2,l+4,l+3)]
        
        return self.create_obj(verts,faces)

    def add_separator(self,tip_y,xz_radius,h_angle,up_angle,down_angle):
        verts = []
        faces = []

        # create the end of the cylinder
        verts += [(-xz_radius*math.cos(i*self.a_step),4*xz_radius,xz_radius*math.sin(i*self.a_step)) for i in range(self.resolution)]
        faces += [(tuple(range(self.resolution)))]

        final_up_r = tip_y*math.sin(up_angle)
        final_up_y = tip_y*math.cos(up_angle)
        final_h_r = tip_y*math.sin(h_angle)
        final_h_y = tip_y*math.cos(h_angle)
        final_down_r = tip_y*math.sin(down_angle)
        final_down_y = tip_y*math.cos(down_angle)
        
        num_verts_above_below = math.floor((self.resolution-1)/2)
        
        # create the large side of the cone by finding the points
        # that intersect a cylinder of radius xz_radius
        l = len(verts)-1
        pt_h_y = tip_y-(xz_radius-final_h_r)*math.tan(h_angle)
        pt_up_y = tip_y*math.cos(up_angle)-(xz_radius-final_up_r)*math.tan(up_angle)
        pt_down_y = tip_y*math.cos(down_angle)-(xz_radius-final_down_r)*math.tan(down_angle)
        
        for i in range(num_verts_above_below+1):
            a = i*self.a_step
            x = xz_radius*math.cos(a)
            z = xz_radius*math.sin(a)
            y_v = math.sqrt(math.pow(tip_y,2)-math.pow(z,2)) if z < final_up_r else final_up_y-(z-final_up_r)*math.tan(up_angle)
            y_h = math.sqrt(math.pow(tip_y,2)-math.pow(x,2)) if abs(x) < final_h_r else final_h_y-(abs(x)-final_h_r)*math.tan(h_angle)
            y = tip_y-(tip_y-y_h)*abs(math.cos(a))-(tip_y-y_v)*math.sin(a)
            verts += [(-x,y,z)]
        for i in range(num_verts_above_below+1):
            a = i*self.a_step
            x = xz_radius*math.cos(a)
            z = xz_radius*math.sin(a)
            y_v = math.sqrt(math.pow(tip_y,2)-math.pow(z,2)) if z < final_down_r else final_down_y-(z-final_down_r)*math.tan(down_angle)
            y_h = math.sqrt(math.pow(tip_y,2)-math.pow(x,2)) if abs(x) < final_h_r else final_h_y-(abs(x)-final_h_r)*math.tan(h_angle)
            y = tip_y-(tip_y-y_h)*abs(math.cos(a))-(tip_y-y_v)*math.sin(a)
            verts += [(x,y,-z)]
        
        self.bridge_last_loop(verts,faces)

        # create the small side of the cone by finding the points
        # that intersect a sphere of radius tip_y
        a_h_max = max(math.asin(final_h_r/tip_y),0.001)
        a_h_step = 4*a_h_max/self.resolution
        a_up_max = math.asin(final_up_r/tip_y)
        for i in range(num_verts_above_below+1):
            a = -a_h_max+i*a_h_step
            b = a_up_max*math.cos(a/a_h_max*math.pi/2)
            verts += [(tip_y*math.cos(b)*math.sin(a),tip_y*math.cos(b)*math.cos(a),tip_y*math.sin(b))]

        a_down_max = math.asin(final_down_r/tip_y)
        for i in range(num_verts_above_below+1):
            a = -a_h_max+i*a_h_step
            b = a_down_max*math.cos(a/a_h_max*math.pi/2)
            verts += [(-tip_y*math.cos(b)*math.sin(a),tip_y*math.cos(b)*math.cos(a),-tip_y*math.sin(b))]
        
        self.bridge_last_loop(verts,faces)

        # create the spherical part
        max_angle = max(a_h_max,a_up_max,a_down_max)
        num_rings = math.floor(max_angle/self.a_step)
        if num_rings > 0:
            a_h_step = a_h_max/num_rings
            a_up_step = a_up_max/num_rings
            a_down_step = a_down_max/num_rings
            
            for i in range(num_rings-1,0,-1):
                ah = i*a_h_step
                au = i*a_up_step
                ad = i*a_down_step
                for j in range(num_verts_above_below+1):
                    a = -ah+j*4*ah/self.resolution
                    b = au*math.cos(a/ah*math.pi/2)
                    verts += [(tip_y*math.cos(b)*math.sin(a),tip_y*math.cos(b)*math.cos(a),tip_y*math.sin(b))]
                for j in range(num_verts_above_below+1):
                    a = -ah+j*4*ah/self.resolution
                    b = ad*math.cos(a/ah*math.pi/2)
                    verts += [(-tip_y*math.cos(b)*math.sin(a),tip_y*math.cos(b)*math.cos(a),-tip_y*math.sin(b))]
                self.bridge_last_loop(verts,faces)
        
        # create the tip
        verts += [(0,tip_y,0)]
        l = len(verts)-1
        faces += [(l,l-i,l-i-1) for i in range(1,self.resolution)]
        faces += [(l,l-self.resolution,l-1)]

        return self.create_obj(verts,faces)

    
    def apply_boolean(self,objA,objB,type,solver='MANIFOLD'):
        # There doesn't appear to be a way to do boolean operations with bmesh
        bpy.context.view_layer.objects.active = objA
        bool = objA.modifiers.new(name='bool',type='BOOLEAN')
        bool.object = objB
        bool.operation = type
        bool.solver = solver
        bpy.ops.object.modifier_apply(modifier=bool.name)
        bpy.data.objects.remove(objB)
    
    def add_floor(self):
        verts = []
        faces = []
        
        x = self.separator_outer_r
        y = 4*self.separator_y+self.clearance
        z1 = -self.raise_z-1
        z2 = -self.separator_outer_r-self.clearance
        
        verts += [(x,y,z1),(-x,y,z1),(-x,-y,z1),(x,-y,z1)]
        faces += [(0,1,2,3)]
        verts += [(x,y,z2),(-x,y,z2),(-x,-y,z2),(x,-y,z2)]
        faces += [(7,6,5,4)]
        faces += [(1,0,4,5),(2,1,5,6),(3,2,6,7),(0,3,7,4)]
        
        return self.create_obj(verts,faces)
    
    def joint1side(self):
        a = self.add_arm_space()
        b = self.add_separator(self.separator_y,self.separator_outer_r,self.print_angle,self.print_angle,self.print_angle)
        self.apply_boolean(a,b,"UNION")
        b = self.add_socket()
        self.apply_boolean(a,b,"UNION")
        a_h = self.print_angle-self.bendable_angle
        if a_h <= 0:
            a_h = math.radians(1)
        a_u = self.print_angle-self.up_angle
        if a_u <= 0:
            a_u = math.radians(1)
        a_d = self.print_angle-self.down_angle
        if a_d <= 0:
            a_d = math.radians(1)
        b = self.add_separator(self.separator_y+self.clearance,2*self.separator_outer_r,a_h,a_u,a_d)
        self.apply_boolean(a,b,"DIFFERENCE")

        b = self.add_floor()
        self.apply_boolean(a,b,"DIFFERENCE")
        
        b = self.add_arm()
        self.apply_boolean(a,b,"DIFFERENCE")
        b = self.add_axle()
        self.apply_boolean(a,b,"DIFFERENCE")
        
        a.location.z += self.raise_z
        
        # clean up the boolean operation
        bpy.context.view_layer.objects.active=a
        bpy.ops.object.editmode_toggle()
        bpy.ops.mesh.select_all(action='SELECT')
        bpy.ops.mesh.dissolve_degenerate(threshold=0.001)
        bpy.ops.object.editmode_toggle()
        return a
    
    def joint2side(self):
        a_min = min(self.bendable_angle,self.up_angle,self.down_angle)
        
        y = self.separator_y
        y_offset = Vector((0,y,0))#+self.clearance/4,0))

        rot180 = Vector((0,0,math.pi))
        a = self.add_separator(self.separator_y,self.separator_outer_r,self.bendable_angle,self.up_angle,self.down_angle)
        a.location -= y_offset
        # the following adds self.clearance to have a cleaner boolean operation
        b = self.add_separator(self.separator_y,self.separator_outer_r+self.clearance,self.bendable_angle,self.up_angle,self.down_angle)
        b.rotation_euler = rot180
        b.location += y_offset
        self.apply_boolean(a,b,"INTERSECT")
        
        b = self.add_arm_space(y+self.clearance/100)
        b.location -= y_offset
        self.apply_boolean(a,b,"UNION")
        b = self.add_arm_space(y+self.clearance/100)
        b.rotation_euler = rot180
        b.location += y_offset
        self.apply_boolean(a,b,"UNION")
        
        b = self.add_socket()
        b.location -= y_offset
        self.apply_boolean(a,b,"UNION")
        b = self.add_socket()
        b.location += y_offset
        self.apply_boolean(a,b,"UNION")
        
        b = self.add_floor()
        self.apply_boolean(a,b,"DIFFERENCE")
        
        b = self.add_arm()
        b.location -= y_offset
        self.apply_boolean(a,b,"DIFFERENCE")
        b = self.add_arm()
        b.rotation_euler = rot180
        b.location += y_offset
        self.apply_boolean(a,b,"DIFFERENCE")
        
        b = self.add_axle()
        b.location -= y_offset
        self.apply_boolean(a,b,"DIFFERENCE")
        b = self.add_axle()
        b.location += y_offset
        self.apply_boolean(a,b,"DIFFERENCE")

        a.location.z += self.raise_z
        
        # clean up the boolean operation
        bpy.context.view_layer.objects.active=a
        bpy.ops.object.editmode_toggle()
        bpy.ops.mesh.select_all(action='SELECT')
        bpy.ops.mesh.dissolve_degenerate(threshold=0.001)
        bpy.ops.object.editmode_toggle()
        return a
    
    def joint(self):
        # make sure nothing is selected
        for obj in bpy.context.selected_objects[:]:
            obj.select_set(False)
        ret = self.joint1side() if self.type == "1SIDE" else self.joint2side()
        ret.select_set(True)
        bpy.context.view_layer.objects.active = ret
        return ret
        

class OBJECT_OT_fdmjoint(Operator):
    """Create a new Joint"""
    bl_idname = "mesh.fdmjoint"
    bl_label = "Add FDM Joint"
    bl_options = {'REGISTER', 'UNDO'}
    
    type: EnumProperty(
        name="Separation Type",
        description="The type of separation between joints",
        items=[
            ("1SIDE","1 Sided","",1),
            ("2SIDE","2 Sided","",2)
        ],
        default="1SIDE"
    )
    
    socket_width: FloatProperty(
        name="Socket Width",
        description="The width of the joint socket",
        subtype="DISTANCE",
        default=4,
        min=2.8
    )
    
    up_angle: FloatProperty(
        name="Up Angle",
        description="The upward bending angle",
        subtype="ANGLE",
        min=0,
        max=math.radians(45),
        default=math.radians(22.5),
        step=100
    )
    
    down_angle: FloatProperty(
        name="Down Angle",
        description="The downward bending angle",
        subtype="ANGLE",
        min=0,
        max=math.radians(45),
        default=math.radians(22.5),
        step=100
    )
    
    horizontal_angle: FloatProperty(
        name="Horizontal Angle Limit",
        description="Limits the horizontal angle, but has no effect if the joint cannot move that far anyway",
        subtype="ANGLE",
        min=0,
        max=math.radians(45),
        default=math.radians(45),
        step=100
    )
    
    clearance: FloatProperty(
        name="Clearance",
        description="The print clearance between moving parts",
        subtype="DISTANCE",
        default=0.3,
        min=0.1
    )
    
    resolution: IntProperty(
        name="Resolution",
        description="The resolution of the curved parts.",
        min=8,
        step=4,
        default=64
    )
    
    location: FloatVectorProperty(name="Location",unit="LENGTH")
    
    rotation: FloatVectorProperty(name="Rotation",unit="ROTATION")

    def execute(self, context):

        try:
            j = HPaulsen_FDMJoint(self.type,self.socket_width,self.up_angle,self.down_angle,self.horizontal_angle,self.clearance,self.resolution)
            obj = j.joint()
            obj.location = obj.location+Vector(self.location)
            obj.rotation_euler = Vector(self.rotation)
        except RuntimeError as err:
            self.report({'ERROR'},str(err))

        return {'FINISHED'}

# Registration

def fdmjoint_button(self, context):
    self.layout.operator(
        OBJECT_OT_fdmjoint.bl_idname,
        text="FDM Joint",
        icon='PLUGIN')

# This allows you to right click on a button and link to documentation
def fdmjoint_manual_map():
    url_manual_prefix = "https://github.com/hpaulsen/fdm_joints/"
    url_manual_mapping = (
        ("bpy.ops.mesh.FDMJoint", "scene_layout/object/types.html"),
    )
    return url_manual_prefix, url_manual_mapping

def register():
    bpy.utils.register_class(OBJECT_OT_fdmjoint)
    bpy.utils.register_manual_map(fdmjoint_manual_map)
    bpy.types.VIEW3D_MT_mesh_add.append(fdmjoint_button)

def unregister():
    bpy.utils.unregister_class(OBJECT_OT_fdmjoint)
    bpy.utils.unregister_manual_map(fdmjoint_manual_map)
    bpy.types.VIEW3D_MT_mesh_add.remove(fdmjoint_button)

if __name__ == "__main__":
    register()