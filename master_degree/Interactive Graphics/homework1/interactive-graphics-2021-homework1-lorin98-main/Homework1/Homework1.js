"use strict";

var shadedPolygon = function() {

var canvas;
var gl;

// ln 22 faces, each face has 2 triangles, each triangle has 3 vertices,
// ln therefore:
var numPositions = 132;

var texSize = 32;

// ln here there is the definition of the bump texture:
// 1) bump data
// 2) compute bump normals
// 3) scale to texture coordinates
// ln 4) get the final output of the normal texture

// ln 1) is there a rougher surface than a random surface?
var data = new Array()
    for (var i = 0; i<= texSize; i++)  data[i] = new Array();
    for (var i = 0; i<= texSize; i++) for (var j=0; j<=texSize; j++)
        data[i][j] = Math.random();

// ln 2)
var normalst = new Array()
    for (var i=0; i<texSize; i++)  normalst[i] = new Array();
    for (var i=0; i<texSize; i++) for ( var j = 0; j < texSize; j++)
        normalst[i][j] = new Array();
    for (var i=0; i<texSize; i++) for ( var j = 0; j < texSize; j++) {
        normalst[i][j][0] = data[i][j]-data[i+1][j];
        normalst[i][j][1] = data[i][j]-data[i][j+1];
        normalst[i][j][2] = 1;
    }

    // ln 3)
    for (var i=0; i<texSize; i++) for (var j=0; j<texSize; j++) {
       var d = 0;
       for(k=0;k<3;k++) d+=normalst[i][j][k]*normalst[i][j][k];
       d = Math.sqrt(d);
       for(k=0;k<3;k++) normalst[i][j][k]= 0.5*normalst[i][j][k]/d + 0.5;
    }

// ln 4)
var normals = new Uint8Array(3*texSize*texSize);

    for ( var i = 0; i < texSize; i++ )
        for ( var j = 0; j < texSize; j++ )
           for(var k =0; k<3; k++)
                normals[3*texSize*i+3*j+k] = 255*normalst[i][j][k];


// ln by default the bump texture is deactivated, therefore:
var applyTexture = false;

var positionsArray = [];
var normalsArray = [];
var texCoordsArray = [];

// ln I need another array to store the tangents for the bump mapping
var tangentsArray = [];

var texCoord = [
   vec2(0, 0),
   vec2(0, 1),
   vec2(1, 1),
   vec2(1, 0)
];

// ln 24 total vertices
var vertices = [
        vec4(-0.5, -0.5,  0.5, 1.0),
        vec4(-0.5,  0.5,  0.5, 1.0),
        vec4(0.5,  0.5,  0.5, 1.0),
        vec4(0.5, -0.5,  0.5, 1.0),
        vec4(-0.5, -0.5, -0.5, 1.0),
        vec4(-0.5,  0.5, -0.5, 1.0),
        vec4(0.5,  0.5, -0.5, 1.0),
        vec4(0.5, -0.5, -0.5, 1.0),
        // ln my added vertices
        vec4(-0.25, -0.25, 0.75, 1.0), // 8
        vec4(-0.25, 0.25, 0.75, 1.0), // 9
        vec4(0.25, 0.25, 0.75, 1.0), // 10
        vec4(0.25, -0.25, 0.75, 1.0), // 11

        vec4(-0.25, -0.25, -0.75, 1.0), // 12
        vec4(-0.25, 0.25, -0.75, 1.0), // 13
        vec4(0.25,  0.25, -0.75, 1.0), // 14
        vec4(0.25, -0.25, -0.75, 1.0), // 15

        vec4(-0.9, -0.25, -0.25, 1.0), // 16
        vec4(-0.9, -0.25, 0.25, 1.0), // 17
        vec4(-0.9, 0.25, 0.25, 1.0), // 18
        vec4(-0.9, 0.25, -0.25, 1.0), // 19

        vec4(1.2, -0.25, -0.25, 1.0), // 20
        vec4(1.2, -0.25, 0.25, 1.0), // 21
        vec4(1.2, 0.25, 0.25, 1.0), // 22
        vec4(1.2, 0.25, -0.25, 1.0), // 23
    ];


// ln barycenter of the polygon, here I do a basic approximation:
// ln the trapezoid on the right makes my barycenter moves only on the x-axis
// ln of a quantity that can be approximated to (1.2 + (-0.9)) / 2
var barycenter = vec3(0.15, 0.0, 0.0);
var bar_rotation = true;

var lightPosition = vec4(1.0, 1.0, 1.0, 0.0);
var lightAmbient = vec4(0.2, 0.2, 0.2, 1.0);
var lightDiffuse = vec4(1.0, 1.0, 1.0, 1.0);
var lightSpecular = vec4(1.0, 1.0, 1.0, 1.0);

// ln properties of the GOLD material, according to:
// ln http://devernay.free.fr/cours/opengl/materials.html
var materialAmbient = vec4(0.24725, 0.1995, 0.0745, 1.0);
var materialDiffuse = vec4(0.75164, 0.600648, 0.22648, 1.0);
var materialSpecular = vec4(0.628281, 0.555802, 0.366065, 1.0);
var materialShininess = 0.4 * 128;

var modelViewMatrix, projectionMatrix;
var modelViewMatrixLoc, projectionMatrixLoc;

var nMatrix, nMatrixLoc;

var viewerPos;
const at = vec3(0.0, 0.0, 0.0);
const up = vec3(0.0, 1.0, 0.0);

var program;

// ln program for the cylinder
var program_cyl;

// ln default values for perspective projection
var near = -1.0;
var far = 3.0;
var radius = 4.0;
// ln value of the theta ANGLE, please do not confuse it with
// the theta VECTOR below that handles rotations around the axes
var thetaVal = 0.0;
var phi = 0.0;
var  fovy = 45.0;
var  aspect = 1.0;

var xAxis = 0;
var yAxis = 1;
var zAxis = 2;
var axis = 0;
var theta = vec3(0, 0, 0);

var flag = false;

// ln variable to switch between per vertex and per fragment shading
// default is per vertex, therefore:
var perFragment = false;

// ln variable to handle cylindrical light
// default is light off, therefore:
var lightOn = false;

init();

function configureTexture( image ) {
   var texture = gl.createTexture();
   gl.activeTexture(gl.TEXTURE0);
   gl.bindTexture(gl.TEXTURE_2D, texture);
   gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGB, texSize, texSize, 0, gl.RGB, gl.UNSIGNED_BYTE, image);
   gl.generateMipmap(gl.TEXTURE_2D);
}

// ln for each quad, I associate to the vertices:
// ln - 4D position coordinates
// ln - 3D normal coordinates
// ln - 2D texture coordinates
// ln +
// ln - 3D tangent coordinates for the bump mapping
function quad(a, b, c, d) {
     // ln standard 3-dimensional normal associated to each vertex
     var t1 = subtract(vertices[b], vertices[a]);
     var t2 = subtract(vertices[c], vertices[b]);
     var normal = cross(t1, t2);
     normal = vec3(normal);
   
     // ln tangent vector is orthogonal to the normal
     // ln I can therefore imagine the tangent as one segment of the quad
     var tangent = vec3(t1[0], t1[1], t1[2]);
     
     //console.log(tangent[0], tangent[1], tangent[2]);

     // ln pushing also the tangent
     positionsArray.push(vertices[a]);
     normalsArray.push(normal);
     texCoordsArray.push(texCoord[0]);
     tangentsArray.push(tangent);

     positionsArray.push(vertices[b]);
     normalsArray.push(normal);
     texCoordsArray.push(texCoord[1]);
     tangentsArray.push(tangent);

     positionsArray.push(vertices[c]);
     normalsArray.push(normal);
     texCoordsArray.push(texCoord[2]);
     tangentsArray.push(tangent);

     positionsArray.push(vertices[a]);
     normalsArray.push(normal);
     texCoordsArray.push(texCoord[0]);
     tangentsArray.push(tangent);

     positionsArray.push(vertices[c]);
     normalsArray.push(normal);
     texCoordsArray.push(texCoord[2]);
     tangentsArray.push(tangent);

     positionsArray.push(vertices[d]);
     normalsArray.push(normal);
     texCoordsArray.push(texCoord[3]);
     tangentsArray.push(tangent);
}

// ln since my geometry is created starting from the cube,
// ln I need to remove some pre-existing quads and add some others
// ln in order to include the trapezoidal shapes on its sides
function colorPolygon()
{
    //quad(1, 0, 3, 2);
    //quad(2, 3, 7, 6);
    quad(3, 0, 4, 7);
    quad(6, 5, 1, 2);
    //quad(4, 5, 6, 7);
    //quad(5, 4, 0, 1);
    
    // ln my added quads
    quad(9, 8, 11, 10);
    quad(1, 0, 8, 9);
    quad(8, 0, 3, 11);
    quad(11, 3, 2, 10);
    quad(1, 9, 10, 2);

    quad(12, 13, 14, 15);
    quad(4, 5, 13, 12);
    quad(4, 12, 15, 7);
    quad(15, 14, 6, 7);
    quad(13, 5, 6, 14);
    
    quad(18, 19, 16, 17);
    quad(17, 16, 4, 0);
    quad(5, 19, 18, 1);
    quad(17, 0, 1, 18);
    quad(16, 19, 5, 4);
    
    quad(20, 23, 22, 21);
    quad(21, 3, 7, 20);
    quad(20, 7, 6, 23);
    quad(23, 6, 2, 22);
    quad(22, 2, 3, 21);
}

// ln practical function to change text on a button
function changeButtonText(id, text)
{
    var elem = document.getElementById(id);
    elem.value = text;
}


// ln cylinder function taken from Code_update/09/geometry.js
function cylinder(numSlices, numStacks, caps) {

var slices = 36;
if(numSlices) slices = numSlices;
var stacks = 1;
if(numStacks) stacks = numStacks;
var capsFlag = true;
if(caps==false) capsFlag = caps;

var data = {};

var top = 0.5;
var bottom = -0.5;
var radius = 0.5;
var topCenter = [0.0, top, 0.0];
var bottomCenter = [0.0, bottom, 0.0];


var sideColor = [1.0, 0.0, 0.0, 1.0];
var topColor = [0.0, 1.0, 0.0, 1.0];
var bottomColor = [0.0, 0.0, 1.0, 1.0];


var cylinderVertexCoordinates = [];
var cylinderNormals = [];
var cylinderVertexColors = [];
var cylinderTextureCoordinates = [];

// side

for(var j=0; j<stacks; j++) {
  var stop = bottom + (j+1)*(top-bottom)/stacks;
  var sbottom = bottom + j*(top-bottom)/stacks;
  var topPoints = [];
  var bottomPoints = [];
  var topST = [];
  var bottomST = [];
  for(var i =0; i<slices; i++) {
    var theta = 2.0*i*Math.PI/slices;
    topPoints.push([radius*Math.sin(theta), stop, radius*Math.cos(theta), 1.0]);
    bottomPoints.push([radius*Math.sin(theta), sbottom, radius*Math.cos(theta), 1.0]);
  };

  topPoints.push([0.0, stop, radius, 1.0]);
  bottomPoints.push([0.0,  sbottom, radius, 1.0]);


  for(var i=0; i<slices; i++) {
    var a = topPoints[i];
    var d = topPoints[i+1];
    var b = bottomPoints[i];
    var c = bottomPoints[i+1];
    var u = [b[0]-a[0], b[1]-a[1], b[2]-a[2]];
    var v = [c[0]-b[0], c[1]-b[1], c[2]-b[2]];

    var normal = [
      u[1]*v[2] - u[2]*v[1],
      u[2]*v[0] - u[0]*v[2],
      u[0]*v[1] - u[1]*v[0]
    ];

    var mag = Math.sqrt(normal[0]*normal[0] + normal[1]*normal[1] + normal[2]*normal[2])
    normal = [normal[0]/mag, normal[1]/mag, normal[2]/mag];
    cylinderVertexCoordinates.push([a[0], a[1], a[2], 1.0]);
    cylinderVertexColors.push(sideColor);
    cylinderNormals.push([normal[0], normal[1], normal[2]]);
    cylinderTextureCoordinates.push([(i+1)/slices, j*(top-bottom)/stacks]);

    cylinderVertexCoordinates.push([b[0], b[1], b[2], 1.0]);
    cylinderVertexColors.push(sideColor);
    cylinderNormals.push([normal[0], normal[1], normal[2]]);
    cylinderTextureCoordinates.push([i/slices, (j-1)*(top-bottom)/stacks]);

    cylinderVertexCoordinates.push([c[0], c[1], c[2], 1.0]);
    cylinderVertexColors.push(sideColor);
    cylinderNormals.push([normal[0], normal[1], normal[2]]);
    cylinderTextureCoordinates.push([(i+1)/slices, (j-1)*(top-bottom)/stacks]);

    cylinderVertexCoordinates.push([a[0], a[1], a[2], 1.0]);
    cylinderVertexColors.push(sideColor);
    cylinderNormals.push([normal[0], normal[1], normal[2]]);
    cylinderTextureCoordinates.push([(i+1)/slices, j*(top-bottom)/stacks]);

    cylinderVertexCoordinates.push([c[0], c[1], c[2], 1.0]);
    cylinderVertexColors.push(sideColor);
    cylinderNormals.push([normal[0], normal[1], normal[2]]);
    cylinderTextureCoordinates.push([(i+1)/slices, (j-1)*(top-bottom)/stacks]);

    cylinderVertexCoordinates.push([d[0], d[1], d[2], 1.0]);
    cylinderVertexColors.push(sideColor);
    cylinderNormals.push([normal[0], normal[1], normal[2]]);
    cylinderTextureCoordinates.push([(i+1)/slices, j*(top-bottom)/stacks]);
  };
};

  var topPoints = [];
  var bottomPoints = [];
  for(var i =0; i<slices; i++) {
    var theta = 2.0*i*Math.PI/slices;
    topPoints.push([radius*Math.sin(theta), top, radius*Math.cos(theta), 1.0]);
    bottomPoints.push([radius*Math.sin(theta), bottom, radius*Math.cos(theta), 1.0]);
  };
  topPoints.push([0.0, top, radius, 1.0]);
  bottomPoints.push([0.0,  bottom, radius, 1.0]);

if(capsFlag) {

//top

for(i=0; i<slices; i++) {
  normal = [0.0, 1.0, 0.0];
  var a = [0.0, top, 0.0, 1.0];
  var b = topPoints[i];
  var c = topPoints[i+1];
  cylinderVertexCoordinates.push([a[0], a[1], a[2], 1.0]);
  cylinderVertexColors.push(topColor);
  cylinderNormals.push(normal);
  cylinderTextureCoordinates.push([0, 1]);

  cylinderVertexCoordinates.push([b[0], b[1], b[2], 1.0]);
  cylinderVertexColors.push(topColor);
  cylinderNormals.push(normal);
  cylinderTextureCoordinates.push([0, 1]);

  cylinderVertexCoordinates.push([c[0], c[1], c[2], 1.0]);
  cylinderVertexColors.push(topColor);
  cylinderNormals.push(normal);
  cylinderTextureCoordinates.push([0, 1]);
};

//bottom

for(i=0; i<slices; i++) {
  normal = [0.0, -1.0, 0.0];
  var a = [0.0, bottom, 0.0, 1.0];
  var b = bottomPoints[i];
  var c = bottomPoints[i+1];
  cylinderVertexCoordinates.push([a[0], a[1], a[2], 1.0]);
  cylinderVertexColors.push(bottomColor);
  cylinderNormals.push(normal);
  cylinderTextureCoordinates.push([0, 1]);

  cylinderVertexCoordinates.push([b[0], b[1], b[2], 1.0]);
  cylinderVertexColors.push(bottomColor);
  cylinderNormals.push(normal);
  cylinderTextureCoordinates.push([0, 1]);

  cylinderVertexCoordinates.push([c[0], c[1], c[2], 1.0]);
  cylinderVertexColors.push(bottomColor);
  cylinderNormals.push(normal);
  cylinderTextureCoordinates.push([0, 1]);
};

};
function translate(x, y, z){
    for(var i=0; i<cylinderVertexCoordinates.length; i++) {
      cylinderVertexCoordinates[i][0] += x;
      cylinderVertexCoordinates[i][1] += y;
      cylinderVertexCoordinates[i][2] += z;
    };
}

function scale(sx, sy, sz){
    for(var i=0; i<cylinderVertexCoordinates.length; i++) {
        cylinderVertexCoordinates[i][0] *= sx;
        cylinderVertexCoordinates[i][1] *= sy;
        cylinderVertexCoordinates[i][2] *= sz;
        cylinderNormals[i][0] /= sx;
        cylinderNormals[i][1] /= sy;
        cylinderNormals[i][2] /= sz;
    };
}

function radians( degrees ) {
    return degrees * Math.PI / 180.0;
}

function rotate( angle, axis) {

    var d = Math.sqrt(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]);

    var x = axis[0]/d;
    var y = axis[1]/d;
    var z = axis[2]/d;

    var c = Math.cos( radians(angle) );
    var omc = 1.0 - c;
    var s = Math.sin( radians(angle) );

    var mat = [
        [ x*x*omc + c,   x*y*omc - z*s, x*z*omc + y*s ],
        [ x*y*omc + z*s, y*y*omc + c,   y*z*omc - x*s ],
        [ x*z*omc - y*s, y*z*omc + x*s, z*z*omc + c ]
    ];

    for(var i=0; i<cylinderVertexCoordinates.length; i++) {
          var u = [0, 0, 0];
          var v = [0, 0, 0];
          for( var j =0; j<3; j++)
            for( var k =0 ; k<3; k++) {
              u[j] += mat[j][k]*cylinderVertexCoordinates[i][k];
              v[j] += mat[j][k]*cylinderNormals[i][k];
            };
            for( var j =0; j<3; j++) {
              cylinderVertexCoordinates[i][j] = u[j];
              cylinderNormals[i][j] = v[j];
            };
    };
}

data.TriangleVertices = cylinderVertexCoordinates;
data.TriangleNormals = cylinderNormals;
data.TriangleVertexColors = cylinderVertexColors;
data.TextureCoordinates = cylinderTextureCoordinates;
data.rotate = rotate;
data.translate = translate;
data.scale = scale;
return data;

}

// ln light function taken from Code_update/09/geometry.js
// ln but we need to specify the light position
function light0(position) {
  var data = {};
  data.lightPosition = position;
  // ln here I set all these '1' to the B components to render a blue neon light
  data.lightAmbient = vec4(0.0, 0.0, 1.0, 1.0);
  data.lightDiffuse = vec4( 0.0, 0.0, 1.0, 1.0 );
  data.lightSpecular = vec4(0.0, 0.0, 1.0, 1.0 );
  data.lightShineness = 100;
  return data;
}

// ln material function taken from Code_update/09/geometry.js
// ln I will use this to set the properties of the cylinder
function cylMaterial() {
  var data  = {};
  // ln PEARL material for the cylinder to simulate the glass of the lamp,
  // ln according to:
  // ln http://devernay.free.fr/cours/opengl/materials.html
  data.materialAmbient = vec4(0.25, 0.20725, 0.20725, 1.0);
  data.materialDiffuse = vec4(1, 0.829, 0.829, 1.0);
  data.materialSpecular = vec4(0.296648, 0.296648, 0.296648, 1.0);
  data.materialShininess = 0.088 * 128;
  // ln I need to add the emissive property
  data.materialEmissive = vec4(0.1, 0.1, 0.1, 1.0); 
  return data;
}


var ncylinder;
var positionLoc2, normalLoc2, modelViewMatrixLoc2, projectionMatrixLoc2, nMatrixLoc2;
var modelViewMatrix2, nMatrix2;

function init() {
    canvas = document.getElementById("gl-canvas");

    gl = canvas.getContext('webgl2');
    if (!gl) alert( "WebGL 2.0 isn't available");


    gl.viewport(0, 0, canvas.width, canvas.height);
    gl.clearColor(0.8, 0.8, 0.8, 1.0);

    gl.enable(gl.DEPTH_TEST);

    aspect =  canvas.width/canvas.height;

    // ln instantiating the cylinder
    var myCylinder = cylinder(72, 3, true);
    myCylinder.scale(0.25, 2.0, 0.25);
    //myCylinder.rotate(180.0, [ 1, 1, 1]);
    myCylinder.translate(-1.5, 0.0, 0.0);
    // ln number of vertices of my cilynder
    ncylinder = myCylinder.TriangleVertices.length;
    var cylMat = cylMaterial();

    // ln creating my three light sources to be put in the cylinder
    var myLight0 = light0(vec4(-2.0, -0.3, -2.0, 1.0));
    var myLight1 = light0(vec4(-2.0, 0.0, -2.0, 1.0));
    var myLight2 = light0(vec4(-2.0, 0.3, -2.0, 1.0));

    //
    //  Load shaders and initialize attribute buffers
    //
    program = initShaders(gl, "vertex-shader", "fragment-shader");
    
    // ln program for the cylinder
    program_cyl = initShaders(gl, "vertex-shader2", "fragment-shader2");

    colorPolygon();

    // ln the polygon is already instantiated, therefore my arrays are now
    // ln composed as [-polygon vertices- -cylinder vertices-]
    positionsArray = positionsArray.concat(myCylinder.TriangleVertices);
    normalsArray = normalsArray.concat(myCylinder.TriangleNormals);
    texCoordsArray = texCoordsArray.concat(myCylinder.TextureCoordinates);
    
    var nBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, nBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, flatten(normalsArray), gl.STATIC_DRAW);

    var normalLoc = gl.getAttribLocation(program, "aNormal");
    gl.vertexAttribPointer(normalLoc, 3, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(normalLoc);

    normalLoc2 = gl.getAttribLocation(program_cyl, "aNormal");
    gl.vertexAttribPointer(normalLoc2, 3, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(normalLoc2);

    var vBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, vBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, flatten(positionsArray), gl.STATIC_DRAW);

    var positionLoc = gl.getAttribLocation(program, "aPosition");
    gl.vertexAttribPointer(positionLoc, 4, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(positionLoc);

    positionLoc2 = gl.getAttribLocation(program_cyl, "aPosition");
    gl.vertexAttribPointer(positionLoc2, 4, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(positionLoc2);

    // texCoordsArray
    var tBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, tBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, flatten(texCoordsArray), gl.STATIC_DRAW);
    
    var texCoordLoc = gl.getAttribLocation(program, "aTexCoord");
    gl.vertexAttribPointer(texCoordLoc, 2, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(texCoordLoc);
        
    // ln tangentsArray
    var tangBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, tangBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, flatten(tangentsArray), gl.STATIC_DRAW);

    var tangentLoc = gl.getAttribLocation(program, "aTangent");
    gl.vertexAttribPointer(tangentLoc, 3, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(tangentLoc);

    //viewerPos = vec3(0.0, 0.0, -20.0);

    //projectionMatrix = ortho(-1, 1, -1, 1, -100, 100);

    var ambientProduct = mult(lightAmbient, materialAmbient);
    var diffuseProduct = mult(lightDiffuse, materialDiffuse);
    var specularProduct = mult(lightSpecular, materialSpecular);

    // ln defining all the light products I need, they are 9: 3 products for 3 lights
    var ambientProduct1 = mult(myLight0.lightAmbient, materialAmbient);
    var diffuseProduct1 = mult(myLight0.lightDiffuse, materialDiffuse);
    var specularProduct1 = mult(myLight0.lightSpecular, materialSpecular);

    var ambientProduct2 = mult(myLight1.lightAmbient, materialAmbient);
    var diffuseProduct2 = mult(myLight1.lightDiffuse, materialDiffuse);
    var specularProduct2 = mult(myLight1.lightSpecular, materialSpecular);

    var ambientProduct3 = mult(myLight2.lightAmbient, materialAmbient);
    var diffuseProduct3 = mult(myLight2.lightDiffuse, materialDiffuse);
    var specularProduct3 = mult(myLight2.lightSpecular, materialSpecular);

    // ln + other 9 products that will effect on the cylinder
    var ambientProduct4 = mult(myLight0.lightAmbient, cylMat.materialAmbient);
    var diffuseProduct4 = mult(myLight0.lightDiffuse, cylMat.materialDiffuse);
    var specularProduct4 = mult(myLight0.lightSpecular, cylMat.materialSpecular);

    var ambientProduct5 = mult(myLight1.lightAmbient, cylMat.materialAmbient);
    var diffuseProduct5 = mult(myLight1.lightDiffuse, cylMat.materialDiffuse);
    var specularProduct5 = mult(myLight1.lightSpecular, cylMat.materialSpecular);

    var ambientProduct6 = mult(myLight2.lightAmbient, cylMat.materialAmbient);
    var diffuseProduct6 = mult(myLight2.lightDiffuse, cylMat.materialDiffuse);
    var specularProduct6 = mult(myLight2.lightSpecular, cylMat.materialSpecular);

    document.getElementById("ButtonX").onclick = function(){axis = xAxis;};
    document.getElementById("ButtonY").onclick = function(){axis = yAxis;};
    document.getElementById("ButtonZ").onclick = function(){axis = zAxis;};
    document.getElementById("ButtonT").onclick = function(){flag = !flag;};
    document.getElementById("ButtonVF").onclick = function(){
      perFragment = !perFragment;
      if (!perFragment){
        changeButtonText("ButtonVF", "Apply Per Fragment");
      }
      else{
        changeButtonText("ButtonVF", "Apply Per Vertex");
      }
      gl.useProgram(program);
      gl.uniform1i(gl.getUniformLocation(program,
        "uPerFragment"), perFragment);
    };
    document.getElementById("ButtonRot").onclick = function(){
      bar_rotation = !bar_rotation;
      if (bar_rotation){
        changeButtonText("ButtonRot", "Rotate around the Origin");
      }
      else{
        changeButtonText("ButtonRot", "Rotate around the Barycenter");
      }
    };

    document.getElementById("zFarSlider").onchange = function(event) {
      far = event.target.value;
    };
    document.getElementById("zNearSlider").onchange = function(event) {
      near = event.target.value;
    };
    document.getElementById("radiusSlider").onchange = function(event) {
      radius = event.target.value;
    };
    document.getElementById("thetaSlider").onchange = function(event) {
      thetaVal = event.target.value* Math.PI/180.0;
    };
    document.getElementById("phiSlider").onchange = function(event) {
      phi = event.target.value* Math.PI/180.0;
    };
    document.getElementById("aspectSlider").onchange = function(event) {
      aspect = event.target.value;
    };
    document.getElementById("fovSlider").onchange = function(event) {
      fovy = event.target.value;
    };

    document.getElementById("ButtonLight").onclick = function(){
      lightOn = !lightOn;
      if (!lightOn)
        changeButtonText("ButtonLight", "Turn light on");
      else
        changeButtonText("ButtonLight", "Turn light off");
      gl.useProgram(program);
      gl.uniform1f(gl.getUniformLocation(program,
        "uLightOn"), lightOn);
      gl.useProgram(program_cyl);
      gl.uniform1f(gl.getUniformLocation(program_cyl,
        "uLightOn"), lightOn);
    };

    document.getElementById("ButtonTex").onclick = function(){
      applyTexture = !applyTexture;
      if (!applyTexture){
        changeButtonText("ButtonTex", "Activate Bump Texture")

        // ln if not active I enable the other buttons as explained in the report
        document.getElementById("ButtonVF").disabled = false;
        document.getElementById("ButtonLight").disabled = false;
      }
      else {
        changeButtonText("ButtonTex", "Deactivate Bump Texture")

        // ln if active I disable the other buttons as explained in the report
        document.getElementById("ButtonVF").disabled = true;
        document.getElementById("ButtonLight").disabled = true;
      }
      gl.useProgram(program);

      gl.uniform1f(gl.getUniformLocation(program,
        "uBumpTexture"), applyTexture);
      gl.uniform1i( gl.getUniformLocation(program, "uTextureMap"), 0);
    }

    gl.useProgram(program);

    gl.uniform4fv(gl.getUniformLocation(program, "uAmbientProduct"),
       flatten(ambientProduct));
    gl.uniform4fv(gl.getUniformLocation(program, "uDiffuseProduct"),
       flatten(diffuseProduct));
    gl.uniform4fv(gl.getUniformLocation(program, "uSpecularProduct"),
       flatten(specularProduct));
    
    gl.uniform4fv(gl.getUniformLocation(program, "uLightPosition"),
       flatten(lightPosition));
    gl.uniform1f(gl.getUniformLocation(program,
       "uShininess"), materialShininess);
    
    // ln adding also the products of the three added lights with their positions
    gl.uniform4fv(gl.getUniformLocation(program, "uAmbientProduct1"),
       flatten(ambientProduct1));
    gl.uniform4fv(gl.getUniformLocation(program, "uDiffuseProduct1"),
       flatten(diffuseProduct1));
    gl.uniform4fv(gl.getUniformLocation(program, "uSpecularProduct1"),
       flatten(specularProduct1));
    gl.uniform4fv(gl.getUniformLocation(program, "uLightPosition1"),
       flatten(myLight0.lightPosition));
    gl.uniform1f(gl.getUniformLocation(program,
       "uShininess1"), myLight0.lightShineness);

    gl.uniform4fv(gl.getUniformLocation(program, "uAmbientProduct2"),
       flatten(ambientProduct2));
    gl.uniform4fv(gl.getUniformLocation(program, "uDiffuseProduct2"),
       flatten(diffuseProduct2));
    gl.uniform4fv(gl.getUniformLocation(program, "uSpecularProduct2"),
       flatten(specularProduct2));
    gl.uniform4fv(gl.getUniformLocation(program, "uLightPosition2"),
       flatten(myLight1.lightPosition));
    gl.uniform1f(gl.getUniformLocation(program,
       "uShininess2"), myLight1.lightShineness);
 
    gl.uniform4fv(gl.getUniformLocation(program, "uAmbientProduct3"),
       flatten(ambientProduct3));
    gl.uniform4fv(gl.getUniformLocation(program, "uDiffuseProduct3"),
       flatten(diffuseProduct3));
    gl.uniform4fv(gl.getUniformLocation(program, "uSpecularProduct3"),
       flatten(specularProduct3));
    gl.uniform4fv(gl.getUniformLocation(program, "uLightPosition3"),
       flatten(myLight2.lightPosition));
    gl.uniform1f(gl.getUniformLocation(program,
       "uShininess3"), myLight2.lightShineness);


    modelViewMatrixLoc = gl.getUniformLocation(program, "uModelViewMatrix");
    projectionMatrixLoc = gl.getUniformLocation(program, "uProjectionMatrix");
    nMatrixLoc = gl.getUniformLocation(program, "uNormalMatrix");
    
        
    // ln here I have all I need for the cylinder program
    gl.useProgram(program_cyl);

    gl.uniform4fv(gl.getUniformLocation(program_cyl, "uAmbientProduct"),
       flatten(ambientProduct));
    gl.uniform4fv(gl.getUniformLocation(program_cyl, "uDiffuseProduct"),
       flatten(diffuseProduct));
    gl.uniform4fv(gl.getUniformLocation(program_cyl, "uSpecularProduct"),
       flatten(specularProduct));
    
    gl.uniform4fv(gl.getUniformLocation(program_cyl, "uLightPosition"),
       flatten(lightPosition));
    gl.uniform1f(gl.getUniformLocation(program_cyl,
       "uShininess"), materialShininess);

    gl.uniform4fv(gl.getUniformLocation(program_cyl, "uAmbientProduct1"),
       flatten(ambientProduct4));
    gl.uniform4fv(gl.getUniformLocation(program_cyl, "uDiffuseProduct1"),
       flatten(diffuseProduct4));
    gl.uniform4fv(gl.getUniformLocation(program_cyl, "uSpecularProduct1"),
       flatten(specularProduct4));
    gl.uniform4fv(gl.getUniformLocation(program_cyl, "uLightPosition1"),
       flatten(myLight0.lightPosition));
    gl.uniform1f(gl.getUniformLocation(program_cyl,
       "uShininess1"), myLight0.lightShineness);

    gl.uniform4fv(gl.getUniformLocation(program_cyl, "uAmbientProduct2"),
       flatten(ambientProduct5));
    gl.uniform4fv(gl.getUniformLocation(program_cyl, "uDiffuseProduct2"),
       flatten(diffuseProduct5));
    gl.uniform4fv(gl.getUniformLocation(program_cyl, "uSpecularProduct2"),
       flatten(specularProduct5));
    gl.uniform4fv(gl.getUniformLocation(program_cyl, "uLightPosition2"),
       flatten(myLight1.lightPosition));
    gl.uniform1f(gl.getUniformLocation(program_cyl,
       "uShininess2"), myLight1.lightShineness);
 
    gl.uniform4fv(gl.getUniformLocation(program_cyl, "uAmbientProduct3"),
       flatten(ambientProduct6));
    gl.uniform4fv(gl.getUniformLocation(program_cyl, "uDiffuseProduct3"),
       flatten(diffuseProduct6));
    gl.uniform4fv(gl.getUniformLocation(program_cyl, "uSpecularProduct3"),
       flatten(specularProduct6));
    gl.uniform4fv(gl.getUniformLocation(program_cyl, "uLightPosition3"),
       flatten(myLight2.lightPosition));
    gl.uniform1f(gl.getUniformLocation(program_cyl,
       "uShininess3"), myLight2.lightShineness);
      
    // ln sending also the emissive term
    gl.uniform4fv(gl.getUniformLocation(program_cyl, "uEmissive"),
       flatten(cylMat.materialEmissive));

    modelViewMatrixLoc2 = gl.getUniformLocation(program_cyl, "uModelViewMatrix");
    projectionMatrixLoc2 = gl.getUniformLocation(program_cyl, "uProjectionMatrix");
    nMatrixLoc2 = gl.getUniformLocation(program_cyl, "uNormalMatrix");

    render();
}

function render(){

    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

    if(flag) theta[axis] += 2;

    viewerPos = vec3(radius*Math.sin(thetaVal)*Math.cos(phi),
        radius*Math.sin(thetaVal)*Math.sin(phi),
        radius*Math.cos(thetaVal));
    
    modelViewMatrix = lookAt(viewerPos, at , up);

    // ln perspective projection 
    projectionMatrix = perspective(fovy, aspect, near, far);

    nMatrix = normalMatrix(modelViewMatrix, true );
    
    modelViewMatrix2 = modelViewMatrix;
    nMatrix2 = nMatrix;

    // ln if I'm rotating around the barycenter, then I should exploit the formula:
    // ln M = T(p_f) * R(theta) * T(- p_f), where p_f is my barycenter
    if (bar_rotation){
      // ln T(p_f)
      var T_bar = translate(barycenter[0], barycenter[1], barycenter[2]);
      // ln T(- p_f)
      var T_minusbar = translate(-barycenter[0], -barycenter[1], -barycenter[2])

      modelViewMatrix = mult(modelViewMatrix, T_bar)
      
      modelViewMatrix = mult(modelViewMatrix, rotate(theta[xAxis], vec3(1, 0, 0)));
      modelViewMatrix = mult(modelViewMatrix, rotate(theta[yAxis], vec3(0, 1, 0)));
      modelViewMatrix = mult(modelViewMatrix, rotate(theta[zAxis], vec3(0, 0, 1)));
      
      modelViewMatrix = mult(modelViewMatrix, T_minusbar);
    }
    
    // ln else I simply rotate around the origin
    else {
      modelViewMatrix = mult(modelViewMatrix, rotate(theta[xAxis], vec3(1, 0, 0)));
      modelViewMatrix = mult(modelViewMatrix, rotate(theta[yAxis], vec3(0, 1, 0)));
      modelViewMatrix = mult(modelViewMatrix, rotate(theta[zAxis], vec3(0, 0, 1)));
    }
    
    nMatrix = normalMatrix(modelViewMatrix, true );

    //console.log(modelViewMatrix);

    gl.useProgram(program);
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(modelViewMatrix));
    gl.uniformMatrix4fv(projectionMatrixLoc, false, flatten(projectionMatrix));
    gl.uniformMatrix3fv(nMatrixLoc, false, flatten(nMatrix));

    // ln applying texture if requested
    if (applyTexture){
      configureTexture(normals);
    }
    gl.drawArrays(gl.TRIANGLES, 0, numPositions);

    // ln cylinder program rendering routine: I have the 'unrotated' ModelViewMatrix
    // ln and the vertices put in the tail of the positionsArray
    gl.useProgram(program_cyl);

    gl.uniformMatrix4fv(modelViewMatrixLoc2, false, flatten(modelViewMatrix2));
    gl.uniformMatrix4fv(projectionMatrixLoc2, false, flatten(projectionMatrix));
    gl.uniformMatrix3fv(nMatrixLoc2, false, flatten(nMatrix2));

    gl.drawArrays(gl.TRIANGLES, numPositions, ncylinder);

    requestAnimationFrame(render);
}

}

shadedPolygon();
