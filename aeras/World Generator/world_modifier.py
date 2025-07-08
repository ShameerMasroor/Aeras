import xml.etree.ElementTree as ET
import csv
import random

# Example XML data blocks
xml_blocks = {
    'pine1': '''<model name='pine_1'>
      <link name='link_0'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
        </inertial>
        <pose>0 0 0 1.57 -0 0</pose>
        <visual name='visual'>
          <pose>0 0 0 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://pine_1/trunk.obj</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
          <transparency>0</transparency>
          <cast_shadows>1</cast_shadows>
        </visual>
        <collision name='collision'>
          <laser_retro>90</laser_retro>
          <max_contacts>10</max_contacts>
          <pose>0 0 0 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://pine_1/trunk.obj</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1</mu>
                <mu2>1</mu2>
                <fdir1>0 0 0</fdir1>
                <slip1>0</slip1>
                <slip2>0</slip2>
              </ode>
              <torsional>
                <coefficient>1</coefficient>
                <patch_radius>0</patch_radius>
                <surface_radius>0</surface_radius>
                <use_patch_radius>1</use_patch_radius>
                <ode>
                  <slip>0</slip>
                </ode>
              </torsional>
            </friction>
            <bounce>
              <restitution_coefficient>0</restitution_coefficient>
              <threshold>1e+06</threshold>
            </bounce>
            <contact>
              <collide_without_contact>0</collide_without_contact>
              <collide_without_contact_bitmask>1</collide_without_contact_bitmask>
              <collide_bitmask>1</collide_bitmask>
              <ode>
                <soft_cfm>0</soft_cfm>
                <soft_erp>0.2</soft_erp>
                <kp>1e+13</kp>
                <kd>1</kd>
                <max_vel>0.01</max_vel>
                <min_depth>0</min_depth>
              </ode>
              <bullet>
                <split_impulse>1</split_impulse>
                <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
                <soft_cfm>0</soft_cfm>
                <soft_erp>0.2</soft_erp>
                <kp>1e+13</kp>
                <kd>1</kd>
              </bullet>
            </contact>
          </surface>
        </collision>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='link_1'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
        </inertial>
        <pose>0 0 0 1.57 -0 0</pose>
        <visual name='visual'>
          <pose>0 0 0 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://pine_1/crown.obj</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
          <transparency>0</transparency>
          <cast_shadows>1</cast_shadows>
        </visual>
        <collision name='collision'>
          <laser_retro>80</laser_retro>
          <max_contacts>10</max_contacts>
          <pose>0 0 0 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://pine_1/crown.obj</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1</mu>
                <mu2>1</mu2>
                <fdir1>0 0 0</fdir1>
                <slip1>0</slip1>
                <slip2>0</slip2>
              </ode>
              <torsional>
                <coefficient>1</coefficient>
                <patch_radius>0</patch_radius>
                <surface_radius>0</surface_radius>
                <use_patch_radius>1</use_patch_radius>
                <ode>
                  <slip>0</slip>
                </ode>
              </torsional>
            </friction>
            <bounce>
              <restitution_coefficient>0</restitution_coefficient>
              <threshold>1e+06</threshold>
            </bounce>
            <contact>
              <collide_without_contact>0</collide_without_contact>
              <collide_without_contact_bitmask>1</collide_without_contact_bitmask>
              <collide_bitmask>1</collide_bitmask>
              <ode>
                <soft_cfm>0</soft_cfm>
                <soft_erp>0.2</soft_erp>
                <kp>1e+13</kp>
                <kd>1</kd>
                <max_vel>0.01</max_vel>
                <min_depth>0</min_depth>
              </ode>
              <bullet>
                <split_impulse>1</split_impulse>
                <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
                <soft_cfm>0</soft_cfm>
                <soft_erp>0.2</soft_erp>
                <kp>1e+13</kp>
                <kd>1</kd>
              </bullet>
            </contact>
          </surface>
        </collision>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <static>1</static>
      <allow_auto_disable>1</allow_auto_disable>
      <pose>-2.92651 -0.317229 0 0 -0 0</pose>
    </model>
    <model name='pine_1'>
        <pose>-2.92651 -0.317229 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link_0'>
          <pose>-2.92651 -0.317229 0 1.57 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='link_1'>
          <pose>-2.92651 -0.317229 0 1.57 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>''',

    'pine2': '''<model name='pine_2'>
      <link name='link_0'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
        </inertial>
        <pose>-0 0 0 1.57 -0 0</pose>
        <visual name='visual'>
          <pose>0 0 0 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://pine_2/crown.obj</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
          <transparency>0</transparency>
          <cast_shadows>1</cast_shadows>
        </visual>
        <collision name='collision'>
          <laser_retro>80</laser_retro>
          <max_contacts>10</max_contacts>
          <pose>0 0 0 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://pine_2/crown.obj</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1</mu>
                <mu2>1</mu2>
                <fdir1>0 0 0</fdir1>
                <slip1>0</slip1>
                <slip2>0</slip2>
              </ode>
              <torsional>
                <coefficient>1</coefficient>
                <patch_radius>0</patch_radius>
                <surface_radius>0</surface_radius>
                <use_patch_radius>1</use_patch_radius>
                <ode>
                  <slip>0</slip>
                </ode>
              </torsional>
            </friction>
            <bounce>
              <restitution_coefficient>0</restitution_coefficient>
              <threshold>1e+06</threshold>
            </bounce>
            <contact>
              <collide_without_contact>0</collide_without_contact>
              <collide_without_contact_bitmask>1</collide_without_contact_bitmask>
              <collide_bitmask>1</collide_bitmask>
              <ode>
                <soft_cfm>0</soft_cfm>
                <soft_erp>0.2</soft_erp>
                <kp>1e+13</kp>
                <kd>1</kd>
                <max_vel>0.01</max_vel>
                <min_depth>0</min_depth>
              </ode>
              <bullet>
                <split_impulse>1</split_impulse>
                <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
                <soft_cfm>0</soft_cfm>
                <soft_erp>0.2</soft_erp>
                <kp>1e+13</kp>
                <kd>1</kd>
              </bullet>
            </contact>
          </surface>
        </collision>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='link_1'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
        </inertial>
        <pose>0 0 0 1.57 -0 0</pose>
        <visual name='visual'>
          <pose>0 0 0 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://pine_2/trunk.obj</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
          <transparency>0</transparency>
          <cast_shadows>1</cast_shadows>
        </visual>
        <collision name='collision'>
          <laser_retro>90</laser_retro>
          <max_contacts>10</max_contacts>
          <pose>0 0 0 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://pine_2/trunk.obj</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1</mu>
                <mu2>1</mu2>
                <fdir1>0 0 0</fdir1>
                <slip1>0</slip1>
                <slip2>0</slip2>
              </ode>
              <torsional>
                <coefficient>1</coefficient>
                <patch_radius>0</patch_radius>
                <surface_radius>0</surface_radius>
                <use_patch_radius>1</use_patch_radius>
                <ode>
                  <slip>0</slip>
                </ode>
              </torsional>
            </friction>
            <bounce>
              <restitution_coefficient>0</restitution_coefficient>
              <threshold>1e+06</threshold>
            </bounce>
            <contact>
              <collide_without_contact>0</collide_without_contact>
              <collide_without_contact_bitmask>1</collide_without_contact_bitmask>
              <collide_bitmask>1</collide_bitmask>
              <ode>
                <soft_cfm>0</soft_cfm>
                <soft_erp>0.2</soft_erp>
                <kp>1e+13</kp>
                <kd>1</kd>
                <max_vel>0.01</max_vel>
                <min_depth>0</min_depth>
              </ode>
              <bullet>
                <split_impulse>1</split_impulse>
                <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
                <soft_cfm>0</soft_cfm>
                <soft_erp>0.2</soft_erp>
                <kp>1e+13</kp>
                <kd>1</kd>
              </bullet>
            </contact>
          </surface>
        </collision>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <static>1</static>
      <allow_auto_disable>1</allow_auto_disable>
      <pose>-0.54535 0.643649 0 0 -0 0</pose>
    </model>
    <model name='pine_2'>
        <pose>-0.54535 0.643649 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link_0'>
          <pose>-0.54535 0.643649 0 1.57 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='link_1'>
          <pose>-0.54535 0.643649 0 1.57 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>''',

    'pine6': '''<model name='pine_6'>
      <link name='link_0'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
        </inertial>
        <pose>0 0 0 1.57 -0 0</pose>
        <visual name='visual'>
          <pose>0 0 0 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://pine_6/trunk.obj</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
          <transparency>0</transparency>
          <cast_shadows>1</cast_shadows>
        </visual>
        <collision name='collision'>
          <laser_retro>90</laser_retro>
          <max_contacts>10</max_contacts>
          <pose>0 0 0 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://pine_6/trunk.obj</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1</mu>
                <mu2>1</mu2>
                <fdir1>0 0 0</fdir1>
                <slip1>0</slip1>
                <slip2>0</slip2>
              </ode>
              <torsional>
                <coefficient>1</coefficient>
                <patch_radius>0</patch_radius>
                <surface_radius>0</surface_radius>
                <use_patch_radius>1</use_patch_radius>
                <ode>
                  <slip>0</slip>
                </ode>
              </torsional>
            </friction>
            <bounce>
              <restitution_coefficient>0</restitution_coefficient>
              <threshold>1e+06</threshold>
            </bounce>
            <contact>
              <collide_without_contact>0</collide_without_contact>
              <collide_without_contact_bitmask>1</collide_without_contact_bitmask>
              <collide_bitmask>1</collide_bitmask>
              <ode>
                <soft_cfm>0</soft_cfm>
                <soft_erp>0.2</soft_erp>
                <kp>1e+13</kp>
                <kd>1</kd>
                <max_vel>0.01</max_vel>
                <min_depth>0</min_depth>
              </ode>
              <bullet>
                <split_impulse>1</split_impulse>
                <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
                <soft_cfm>0</soft_cfm>
                <soft_erp>0.2</soft_erp>
                <kp>1e+13</kp>
                <kd>1</kd>
              </bullet>
            </contact>
          </surface>
        </collision>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='link_1'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
        </inertial>
        <pose>0 0 0 1.57 -0 0</pose>
        <visual name='visual'>
          <pose>0 0 0 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://pine_6/crown.obj</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
          <transparency>0</transparency>
          <cast_shadows>1</cast_shadows>
        </visual>
        <collision name='collision'>
          <laser_retro>80</laser_retro>
          <max_contacts>10</max_contacts>
          <pose>0 0 0 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://pine_6/crown.obj</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1</mu>
                <mu2>1</mu2>
                <fdir1>0 0 0</fdir1>
                <slip1>0</slip1>
                <slip2>0</slip2>
              </ode>
              <torsional>
                <coefficient>1</coefficient>
                <patch_radius>0</patch_radius>
                <surface_radius>0</surface_radius>
                <use_patch_radius>1</use_patch_radius>
                <ode>
                  <slip>0</slip>
                </ode>
              </torsional>
            </friction>
            <bounce>
              <restitution_coefficient>0</restitution_coefficient>
              <threshold>1e+06</threshold>
            </bounce>
            <contact>
              <collide_without_contact>0</collide_without_contact>
              <collide_without_contact_bitmask>1</collide_without_contact_bitmask>
              <collide_bitmask>1</collide_bitmask>
              <ode>
                <soft_cfm>0</soft_cfm>
                <soft_erp>0.2</soft_erp>
                <kp>1e+13</kp>
                <kd>1</kd>
                <max_vel>0.01</max_vel>
                <min_depth>0</min_depth>
              </ode>
              <bullet>
                <split_impulse>1</split_impulse>
                <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
                <soft_cfm>0</soft_cfm>
                <soft_erp>0.2</soft_erp>
                <kp>1e+13</kp>
                <kd>1</kd>
              </bullet>
            </contact>
          </surface>
        </collision>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <static>1</static>
      <allow_auto_disable>1</allow_auto_disable>
      <pose>2.57468 -1.29623 0 0 -0 0</pose>
    </model>
    <model name='pine_6'>
        <pose>2.57468 -1.29623 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link_0'>
          <pose>2.57468 -1.29623 0 1.57 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='link_1'>
          <pose>2.57468 -1.29623 0 1.57 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>''',

    'pine4': '''<model name='pin_4'>
      <link name='link_0'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
        </inertial>
        <pose>0 0 0 1.57 -0 0</pose>
        <visual name='visual'>
          <pose>0 0 0 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://pine_4/crown.obj</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
          <transparency>0</transparency>
          <cast_shadows>1</cast_shadows>
        </visual>
        <collision name='collision'>
          <laser_retro>80</laser_retro>
          <max_contacts>10</max_contacts>
          <pose>0 0 0 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://pine_4/crown.obj</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1</mu>
                <mu2>1</mu2>
                <fdir1>0 0 0</fdir1>
                <slip1>0</slip1>
                <slip2>0</slip2>
              </ode>
              <torsional>
                <coefficient>1</coefficient>
                <patch_radius>0</patch_radius>
                <surface_radius>0</surface_radius>
                <use_patch_radius>1</use_patch_radius>
                <ode>
                  <slip>0</slip>
                </ode>
              </torsional>
            </friction>
            <bounce>
              <restitution_coefficient>0</restitution_coefficient>
              <threshold>1e+06</threshold>
            </bounce>
            <contact>
              <collide_without_contact>0</collide_without_contact>
              <collide_without_contact_bitmask>1</collide_without_contact_bitmask>
              <collide_bitmask>1</collide_bitmask>
              <ode>
                <soft_cfm>0</soft_cfm>
                <soft_erp>0.2</soft_erp>
                <kp>1e+13</kp>
                <kd>1</kd>
                <max_vel>0.01</max_vel>
                <min_depth>0</min_depth>
              </ode>
              <bullet>
                <split_impulse>1</split_impulse>
                <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
                <soft_cfm>0</soft_cfm>
                <soft_erp>0.2</soft_erp>
                <kp>1e+13</kp>
                <kd>1</kd>
              </bullet>
            </contact>
          </surface>
        </collision>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='link_1'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
        </inertial>
        <pose>0 0 0 1.57 -0 0</pose>
        <visual name='visual'>
          <pose>0 0 0 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://pine_4/trunk.obj</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
          <transparency>0</transparency>
          <cast_shadows>1</cast_shadows>
        </visual>
        <collision name='collision'>
          <laser_retro>90</laser_retro>
          <max_contacts>10</max_contacts>
          <pose>0 0 0 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://pine_4/trunk.obj</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1</mu>
                <mu2>1</mu2>
                <fdir1>0 0 0</fdir1>
                <slip1>0</slip1>
                <slip2>0</slip2>
              </ode>
              <torsional>
                <coefficient>1</coefficient>
                <patch_radius>0</patch_radius>
                <surface_radius>0</surface_radius>
                <use_patch_radius>1</use_patch_radius>
                <ode>
                  <slip>0</slip>
                </ode>
              </torsional>
            </friction>
            <bounce>
              <restitution_coefficient>0</restitution_coefficient>
              <threshold>1e+06</threshold>
            </bounce>
            <contact>
              <collide_without_contact>0</collide_without_contact>
              <collide_without_contact_bitmask>1</collide_without_contact_bitmask>
              <collide_bitmask>1</collide_bitmask>
              <ode>
                <soft_cfm>0</soft_cfm>
                <soft_erp>0.2</soft_erp>
                <kp>1e+13</kp>
                <kd>1</kd>
                <max_vel>0.01</max_vel>
                <min_depth>0</min_depth>
              </ode>
              <bullet>
                <split_impulse>1</split_impulse>
                <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
                <soft_cfm>0</soft_cfm>
                <soft_erp>0.2</soft_erp>
                <kp>1e+13</kp>
                <kd>1</kd>
              </bullet>
            </contact>
          </surface>
        </collision>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <static>1</static>
      <allow_auto_disable>1</allow_auto_disable>
      <pose>1.50587 -2.44231 0 0 -0 0</pose>
    </model>
        <model name='pin_4'>
        <pose>1.50587 -2.44231 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link_0'>
          <pose>1.50587 -2.44231 0 1.57 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='link_1'>
          <pose>1.50587 -2.44231 0 1.57 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>''',

    'pine3': '''<model name='pine_3'>
      <link name='link_0'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
        </inertial>
        <pose>0 0 0 1.57 -0 0</pose>
        <visual name='visual'>
          <pose>0 0 0 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://pine_3/crown.obj</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
          <transparency>0</transparency>
          <cast_shadows>1</cast_shadows>
        </visual>
        <collision name='collision'>
          <laser_retro>80</laser_retro>
          <max_contacts>10</max_contacts>
          <pose>0 0 0 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://pine_3/crown.obj</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1</mu>
                <mu2>1</mu2>
                <fdir1>0 0 0</fdir1>
                <slip1>0</slip1>
                <slip2>0</slip2>
              </ode>
              <torsional>
                <coefficient>1</coefficient>
                <patch_radius>0</patch_radius>
                <surface_radius>0</surface_radius>
                <use_patch_radius>1</use_patch_radius>
                <ode>
                  <slip>0</slip>
                </ode>
              </torsional>
            </friction>
            <bounce>
              <restitution_coefficient>0</restitution_coefficient>
              <threshold>1e+06</threshold>
            </bounce>
            <contact>
              <collide_without_contact>0</collide_without_contact>
              <collide_without_contact_bitmask>1</collide_without_contact_bitmask>
              <collide_bitmask>1</collide_bitmask>
              <ode>
                <soft_cfm>0</soft_cfm>
                <soft_erp>0.2</soft_erp>
                <kp>1e+13</kp>
                <kd>1</kd>
                <max_vel>0.01</max_vel>
                <min_depth>0</min_depth>
              </ode>
              <bullet>
                <split_impulse>1</split_impulse>
                <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
                <soft_cfm>0</soft_cfm>
                <soft_erp>0.2</soft_erp>
                <kp>1e+13</kp>
                <kd>1</kd>
              </bullet>
            </contact>
          </surface>
        </collision>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='link_1'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
        </inertial>
        <pose>0 0 0 1.57 -0 0</pose>
        <visual name='visual'>
          <pose>0 0 0 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://pine_3/trunk.obj</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
          <transparency>0</transparency>
          <cast_shadows>1</cast_shadows>
        </visual>
        <collision name='collision'>
          <laser_retro>90</laser_retro>
          <max_contacts>10</max_contacts>
          <pose>0 0 0 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://pine_3/trunk.obj</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1</mu>
                <mu2>1</mu2>
                <fdir1>0 0 0</fdir1>
                <slip1>0</slip1>
                <slip2>0</slip2>
              </ode>
              <torsional>
                <coefficient>1</coefficient>
                <patch_radius>0</patch_radius>
                <surface_radius>0</surface_radius>
                <use_patch_radius>1</use_patch_radius>
                <ode>
                  <slip>0</slip>
                </ode>
              </torsional>
            </friction>
            <bounce>
              <restitution_coefficient>0</restitution_coefficient>
              <threshold>1e+06</threshold>
            </bounce>
            <contact>
              <collide_without_contact>0</collide_without_contact>
              <collide_without_contact_bitmask>1</collide_without_contact_bitmask>
              <collide_bitmask>1</collide_bitmask>
              <ode>
                <soft_cfm>0</soft_cfm>
                <soft_erp>0.2</soft_erp>
                <kp>1e+13</kp>
                <kd>1</kd>
                <max_vel>0.01</max_vel>
                <min_depth>0</min_depth>
              </ode>
              <bullet>
                <split_impulse>1</split_impulse>
                <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
                <soft_cfm>0</soft_cfm>
                <soft_erp>0.2</soft_erp>
                <kp>1e+13</kp>
                <kd>1</kd>
              </bullet>
            </contact>
          </surface>
        </collision>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <static>1</static>
      <allow_auto_disable>1</allow_auto_disable>
      <pose>23.8617 12.0551 0 0 -0 0</pose>
    </model>
    <model name='pine_3'>
        <pose>23.8617 12.0551 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link_0'>
          <pose>23.8617 12.0551 0 1.57 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='link_1'>
          <pose>23.8617 12.0551 0 1.57 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>'''
}


start_end_code = {
    'start':"""
<sdf version='1.7'>
  <world name='default'>
    <light name='sun' type='directional'>
      <cast_shadows>0</cast_shadows>
      <pose>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
      <spot>
        <inner_angle>0</inner_angle>
        <outer_angle>0</outer_angle>
        <falloff>0</falloff>
      </spot>
    </light>
           <model name="heightmap">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <heightmap>
              <uri>file://media/dem/el_captan_trail1000.tif</uri>
              <size>1000 1000 81.0</size>
              <pos>0 0 -2333</pos>
            </heightmap>
          </geometry>
        </collision>
        <visual name="visual_abcedf">
          <geometry>
            <heightmap>
              <use_terrain_paging>false</use_terrain_paging>
              <texture>
                <diffuse>file://media/materials/textures/dirt_diffusespecular.png</diffuse>
                <normal>file://media/materials/textures/flat_normal.png</normal>
                <size>1</size>
              </texture>
              <texture>
                <diffuse>file://media/materials/textures/grass_diffusespecular.png</diffuse>
                <normal>file://media/materials/textures/flat_normal.png</normal>
                <size>1</size>
              </texture>
              <texture>
                <diffuse>file://media/materials/textures/fungus_diffusespecular.png</diffuse>
                <normal>file://media/materials/textures/flat_normal.png</normal>
                <size>1</size>
              </texture>
              <blend>
                <min_height>2</min_height>
                <fade_dist>5</fade_dist>
              </blend>
              <blend>
                <min_height>4</min_height>
                <fade_dist>5</fade_dist>
              </blend>
              <uri>file://media/dem/el_captan_trail1000.tif</uri>
              <size>1000 1000 81</size>
              <pos>0 0 -2333</pos>
            </heightmap>
          </geometry>
        </visual>
      </link>
    </model>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type='adiabatic'/>
    <physics type="ode">
    <real_time_update_rate>1000</real_time_update_rate>
    <max_step_size>0.001</max_step_size>
    <solver>
        <type>quick</type>
        <iters>100</iters>
        <sor>1.3</sor>
    </solver>
    <thread_position_correction>true</thread_position_correction>
    <max_contacts>20</max_contacts>
  </physics>
    <scene>
      <ambient>0.2 0.2 0.2 1</ambient>
      <background>0.3 0.3 0.3 1</background>
      <shadows>0</shadows>
    </scene>
    <audio>
      <device>default</device>
    </audio>
    <wind/>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>0</latitude_deg>
      <longitude_deg>0</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>""",

    'intermediate': """<state world_name='default'>
      <sim_time>15320 593000000</sim_time>
      <real_time>194 529760342</real_time>
      <wall_time>1733744285 39205164</wall_time>
      <iterations>3993173</iterations>
      <model name='byu'>
        <pose>0 0 0.1 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>0 0 0.001 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='ground_plane'>
        <pose>0 0 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>0 0 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>""",

    'camera': """<gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>-0.831913 19.8282 3.19386 0 0.216 -1.428</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
        <track_visual>
          <name>simple_drone</name>
          <static>1</static>
          <use_model_frame>1</use_model_frame>
          <xyz>-4.5 0 1.5</xyz>
          <inherit_yaw>1</inherit_yaw>
        </track_visual>
      </camera>
    </gui>""", 

    'end': """<model name='rescue_randy_sitting'>
      <static>1</static>
      <frame name='artifact_proximity_detector::__model__' attached_to='artifact_proximity_detector::'>
        <pose relative_to='__model__'>0 0 0 0 -0 0</pose>
      </frame>
      <static>1</static>
      <plugin name='ignition::gazebo::systems::PerformerDetector' filename='libignition-gazebo-performer-detector-system.so'>
        <topic>/subt_performer_detector</topic>
        <header_data>
          <key>type</key>
          <value>artifact_proximity</value>
        </header_data>
        <geometry>
          <box>
            <size>10 10 10</size>
          </box>
        </geometry>
      </plugin>
      <link name='link'>
        <pose>0 0 -0.623996 0 -0 0</pose>
        <collision name='collision'>
          <geometry>
            <mesh>
              <uri>/usr/share/gazebo-11/models/Randy/meshes/rescue_randy.dae</uri>
            </mesh>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <mesh>
              <uri>/usr/share/gazebo-11/models/Randy/meshes/rescue_randy.dae</uri>
            </mesh>
          </geometry>
          <plugin name='ignition::gazebo::systems::Thermal' filename='ignition-gazebo-thermal-system'>
            <heat_signature>materials/textures/RescueRandy_Thermal.png</heat_signature>
            <max_temp>305</max_temp>
          </plugin>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <pose>8.65569 -16.7299 0 0 -0 0</pose>
    </model>
  </world>
</sdf>
"""
    



}

import xml.etree.ElementTree as ET
import csv
import random

def modify_pose(xml_string, new_position, model_name, count):
    # Ensure XML string has only one root
    xml_string = f"<root>{xml_string}</root>"
    tree = ET.ElementTree(ET.fromstring(xml_string))
    root = tree.getroot()
    
    for model in root.findall("model"):
        unique_name = f"{model_name}_{count}" if count > 0 else model_name
        model.set("name", unique_name)  # Set unique name
        
        pose = model.find("pose")  # Only modify the main model pose
        if pose is not None:
            parts = pose.text.split()
            if len(parts) == 6:
                parts[:3] = map(str, new_position)  # Modify only X, Y, Z
                pose.text = ' '.join(parts)
    
    return "\n".join(ET.tostring(model, encoding='unicode') for model in root)

# Read positions from CSV
csv_file = "tree_positions.csv"
selected_models = []
model_counts = {}

def read_positions_from_csv(filename):
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            x, y, z = map(float, row)
            model_name = random.choice(list(xml_blocks.keys()))  # Randomly select a model
            count = model_counts.get(model_name, 0)
            model_counts[model_name] = count + 1
            selected_models.append((model_name, (x, y, z), count))

# Read positions and randomly assign models
read_positions_from_csv(csv_file)

output_file = "coridor.world"

# Modify selected XML blocks and write to file
with open(output_file, 'w') as file:
    file.write(start_end_code['start'])
    for name, position, count in selected_models:
        if name in xml_blocks:
            modified_xml = modify_pose(xml_blocks[name], position, name, count)
            modified_xml = modified_xml.replace('"', "'")  # Convert double quotes to single quotes
            file.write(modified_xml + "\n\n")
    file.write(start_end_code['camera'])
    file.write(start_end_code['end'])
