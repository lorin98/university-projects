"use strict";

var canvas;
var gl;
var program;


var projectionMatrix;
var modelViewMatrix;

var instanceMatrix;

var modelViewMatrixLoc, projectionMatrixLoc;
var nMatrixLoc;

var viewerPos;
const at = vec3(0.0, 0.0, 0.0);
const up = vec3(0.0, 1.0, 0.0);
var radius = 4.0;
var thetaVal = 0.6;
var phi = 30 * Math.PI/180.0;
var left = -10.0;
var right = 10.0;
var bottom = -10.0;
var topp = 10.0;
var near = -10.0;
var far = 40.0;


var vertices = [

    vec4( -0.5, -0.5,  0.5, 1.0 ),
    vec4( -0.5,  0.5,  0.5, 1.0 ),
    vec4( 0.5,  0.5,  0.5, 1.0 ),
    vec4( 0.5, -0.5,  0.5, 1.0 ),
    vec4( -0.5, -0.5, -0.5, 1.0 ),
    vec4( -0.5,  0.5, -0.5, 1.0 ),
    vec4( 0.5,  0.5, -0.5, 1.0 ),
    vec4( 0.5, -0.5, -0.5, 1.0 )
];


// ln flag to start animation
var animation = false;


var torsoId = 0;
var headId  = 1;
var head1Id = 1;
var head2Id = 10;
// ln I have denoted the four legs of the sheep as follows:
// ln the two front legs are "left*Arm" and "right*Arm"
// ln the two rear legs are "left*Leg" and "right*Leg"
var leftUpperArmId = 2;
var leftLowerArmId = 3;
var rightUpperArmId = 4;
var rightLowerArmId = 5;
var leftUpperLegId = 6;
var leftLowerLegId = 7;
var rightUpperLegId = 8;
var rightLowerLegId = 9;
// ln adding tail id
var tailId = 11;


var torsoHeight = 4.0;
var torsoWidth = 2.0;
var lowerArmHeight = 1.0;
var upperArmHeight = 2.0;
var lowerArmWidth  = 0.15;
var upperArmWidth  = 0.25;
var upperLegWidth  = 0.25;
var lowerLegWidth  = 0.15;
var lowerLegHeight = 1.0;
var upperLegHeight = 2.0;
var headHeight = 1.5;
var headWidth = 0.85;
// ln adding tail dimension
var tailHeight = 1.0;
var tailWidth = 0.5;

// ln this variables store and update the x and y coordinates of the torso
// ln since the torso is the root of the tree I have only to update it
// ln the other parts of the sheep will automatically transform
// ln (the z coordinate does not need to be changed)
var torso_x = -7.5;
var torso_y = -6.0;

// ln create the grass field as a separated object
var grassId = 12;
var grassHeight = 2;
var grassWidth = 45;

// ln create the fence as a separated object
var fenceId1 = 13;
var fenceId2 = 14;
var fenceId3 = 15;
var fenceId4 = 16;
var fenceId5 = 17;
var fenceId6 = 18;
var fenceHeight = 5.0;
var fenceWidth = 0.25;

// ln adding two eyes to make the sheep a bit more realistic
var leftEyeId = 19;
var rightEyeId = 20;
var eyeHeight = 0.25;
var eyeWidth = 0.1;

var numNodes = 21;
var numAngles = 11;
var angle = 0;

var theta = [0, 0, -180, 0, -180, 0, 180, 0, 180, 0, 0, -120, 0, 0, 0, 0, 0, 0, 90, 0, 0];

var stack = [];

var figure = [];

for( var i=0; i<numNodes; i++) figure[i] = createNode(null, null, null, null);

var vBuffer;
var modelViewLoc;
var tBuffer;

var pointsArray = [];
var normalsArray = [];
// ln adding texture array
var texCoordsArray = [];
// ln I need another array to store the tangents for the bump mapping
var tangentsArray = [];

// ln lighting params
var lightPosition1 = vec4(-100, 100, 100, 0.0);
var lightPosition2 = vec4(100, 100, -100, 0.0);
var lightAmbient = vec4(0.5, 0.5, 0.5, 1.0);
var lightDiffuse = vec4(1.0, 1.0, 1.0, 1.0);
// ln material params with simplification: only ambient and diffuse
// ln please, read the report for further clarifications
var body_ambient = vec4(0.5, 0.5, 0.5, 0.0);
var body_diffuse = vec4(1.0, 1.0, 1.0, 1.0);
var head_ambient = vec4(0.6, 0.6, 0.6, 0.0);
var head_diffuse = vec4(1.0, 1.0, 1.0, 1.0);
var grass_ambient = vec4(0.0, 0.5, 0.0, 0.0);
var grass_diffuse = vec4(0.5, 0.5, 0.5, 1.0);
var fence_ambient = vec4(0.5451, 0.2701, 0.0745, 0.0);
var fence_diffuse = vec4(0.8, 0.8, 0.8, 1.0);


//-------------------------------------------

function scale4(a, b, c) {
   var result = mat4();
   result[0] = a;
   result[5] = b;
   result[10] = c;
   return result;
}

//--------------------------------------------


function createNode(transform, render, sibling, child){
    var node = {
    transform: transform,
    render: render,
    sibling: sibling,
    child: child,
    }
    return node;
}


function initNodes(Id) {

    var m = mat4();

    switch(Id) {

    case torsoId:

    m = translate(torso_x, torso_y, 0);
    m = mult(m, rotate(theta[torsoId], vec3(0, 0, 1) ));
    figure[torsoId] = createNode( m, torso, null, headId );
    break;

    case headId:
    case head1Id:
    case head2Id:


    m = translate(torsoWidth/2+0.35, torsoHeight-0.25, 0.0);
	m = mult(m, rotate(theta[head1Id], vec3(1, 0, 0)))
	m = mult(m, rotate(theta[head2Id], vec3(0, 0, 1)));
    m = mult(m, translate(0.0, -0.5*headHeight, 0.0));
    figure[headId] = createNode( m, head, leftUpperArmId, leftEyeId);
    break;


    case leftUpperArmId:

    m = translate(torsoWidth/2, -torsoHeight+5, -1.0);
    m = mult(m, rotate(theta[leftUpperArmId], vec3(0, 0, 1)));
    figure[leftUpperArmId] = createNode( m, leftUpperArm, rightUpperArmId, leftLowerArmId );
    break;

    case rightUpperArmId:

    m = translate(torsoWidth/2, -torsoHeight+5, 1.0);
	m = mult(m, rotate(theta[rightUpperArmId], vec3(0, 0, 1)));
    figure[rightUpperArmId] = createNode( m, rightUpperArm, leftUpperLegId, rightLowerArmId );
    break;

    case leftUpperLegId:

    m = translate(-(torsoWidth/2), -torsoHeight+5, -1.0);
	m = mult(m , rotate(theta[leftUpperLegId], vec3(0, 0, 1)));
    figure[leftUpperLegId] = createNode( m, leftUpperLeg, rightUpperLegId, leftLowerLegId );
    break;

    case rightUpperLegId:
    m = translate(-(torsoWidth/2), -torsoHeight+5, 1.0);
    m = mult(m, rotate(theta[rightUpperLegId], vec3(0, 0, 1)));
    figure[rightUpperLegId] = createNode( m, rightUpperLeg, tailId, rightLowerLegId );
    break;

    case leftLowerArmId:

    m = translate(0.0, upperArmHeight, 0.0);
    m = mult(m, rotate(theta[leftLowerArmId], vec3(0, 0, 1)));
    figure[leftLowerArmId] = createNode( m, leftLowerArm, null, null );
    break;

    case rightLowerArmId:

    m = translate(0.0, upperArmHeight, 0.0);
    m = mult(m, rotate(theta[rightLowerArmId], vec3(0, 0, 1)));
    figure[rightLowerArmId] = createNode( m, rightLowerArm, null, null );
    break;

    case leftLowerLegId:

    m = translate(0.0, upperLegHeight, 0.0);
    m = mult(m, rotate(theta[leftLowerLegId],vec3(0, 0, 1)));
    figure[leftLowerLegId] = createNode( m, leftLowerLeg, null, null );
    break;

    case rightLowerLegId:

    m = translate(0.0, upperLegHeight, 0.0);
    m = mult(m, rotate(theta[rightLowerLegId], vec3(0, 0, 1)));
    figure[rightLowerLegId] = createNode( m, rightLowerLeg, null, null );
    break;

    case tailId:
    
    m = translate(-(torsoWidth/2)+0.1, torsoHeight*2/3, 0.0);
    m = mult(m, rotate(theta[tailId], vec3(0, 0, 1)));
    figure[tailId] = createNode( m, tail, null, null );
    break;

    // ln rendering the grass field as a separated object
    case grassId:
    
    m = translate(0, -torsoHeight+3-lowerArmHeight-upperArmHeight-6, 0.0);
    m = mult(m, rotate(theta[grassId], vec3(1, 0, 0)));
    figure[grassId] = createNode( m, grass, null, null );
    break;
    
    // ln rendering the fence as a separated object
    case fenceId1:

    m = translate(0, -torsoHeight-lowerArmHeight-upperArmHeight-1, 0.0);
    m = mult(m, rotate(theta[fenceId1], vec3(1, 0, 0)));
    figure[fenceId1] = createNode( m, fence, fenceId2, null );
    break;

    case fenceId2:

    m = translate(0, -torsoHeight-lowerArmHeight-upperArmHeight-1, 1.0);
    m = mult(m, rotate(theta[fenceId2], vec3(1, 0, 0)));
    figure[fenceId2] = createNode( m, fence, fenceId3, null );

    case fenceId3:

    m = translate(0, -torsoHeight-lowerArmHeight-upperArmHeight-1, -1.0);
    m = mult(m, rotate(theta[fenceId3], vec3(1, 0, 0)));
    figure[fenceId3] = createNode( m, fence, fenceId4, null );

    case fenceId4:

    m = translate(0, -torsoHeight-lowerArmHeight-upperArmHeight-1, -2.0);
    m = mult(m, rotate(theta[fenceId4], vec3(1, 0, 0)));
    figure[fenceId4] = createNode( m, fence, fenceId5, null );

    case fenceId5:

    m = translate(0, -torsoHeight-lowerArmHeight-upperArmHeight-1, 2.0);
    m = mult(m, rotate(theta[fenceId5], vec3(1, 0, 0)));
    figure[fenceId5] = createNode( m, fence, fenceId6, null );

    case fenceId6:

    m = translate(0, -4.0, 2.5);
    m = mult(m, rotate(theta[fenceId6], vec3(1, 0, 0)));
    figure[fenceId6] = createNode( m, fence, null, null );
    
    // ln eye, remember that the left eye is a child of the head
    case leftEyeId:
    m = translate(0.40, 1, -0.25);
    m = mult(m, rotate(theta[leftEyeId], vec3(0, 0, 1)));
    figure[leftEyeId] = createNode( m, eye, rightEyeId, null );

    case rightEyeId:
    m = translate(0.40, 1, 0.25);
    m = mult(m, rotate(theta[rightEyeId], vec3(0, 0, 1)));
    figure[rightEyeId] = createNode( m, eye, null, null );

    break;
    }

}

// ln I have to consider several textures for all the different objects in my scene
var texSize = 64;

// ln function to create a simple color texture, the color must be specified
function createColorTexture(r, g, b){
    var image1 = new Uint8Array([255*r, 255*g, 255*b, 255]);
    return image1;
}

// ln function to create a bump texture (taken from Homework 1)
function createBumpTexture(){
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

    return normals;
}


var sheepBodyImage;
var sheepHeadImage;
var grassImage;
var fenceImage;

// ln standard texture coordinates
var texCoord = [
    vec2(0, 0),
    vec2(0, 1),
    vec2(1, 1),
    vec2(1, 0)
];

// ln variables to render textures
var textureBodySheep;
var textureHeadSheep;
var textureGrass;
var textureFence;

// ln modified version of configureTexture to handle also different images
// ln it is just a sequential texture activation and binding
function configureTexture() {
    // ln body of the sheep
    textureBodySheep = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, textureBodySheep);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGB,texSize, texSize, 0,
         gl.RGB, gl.UNSIGNED_BYTE, sheepBodyImage);
    gl.generateMipmap(gl.TEXTURE_2D);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER,
                      gl.NEAREST_MIPMAP_LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
    
    // ln head of the sheep
    textureHeadSheep = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, textureHeadSheep);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGB,1, 1, 0,
        gl.RGB, gl.UNSIGNED_BYTE, sheepHeadImage);
    gl.generateMipmap(gl.TEXTURE_2D);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER,
                    gl.NEAREST_MIPMAP_LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);

    // ln grass
    textureGrass = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, textureGrass);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGB,
         gl.RGB, gl.UNSIGNED_BYTE, grassImage);
    gl.generateMipmap(gl.TEXTURE_2D);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER,
                      gl.NEAREST_MIPMAP_LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);

    // ln fence
    textureFence = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, textureFence);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGB,
         gl.RGB, gl.UNSIGNED_BYTE, fenceImage);
    gl.generateMipmap(gl.TEXTURE_2D);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER,
                      gl.NEAREST_MIPMAP_LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
    
}

// ln I need also flags to switch between textures according to the "material"
// ln I'm rendering, so:
var sheepBodyText = false;
var sheepHeadText = false;
var grassText = false;
var fenceText = false;
// ln in the rendering functions of the hierarchical structure I have to remember
// ln to properly set these flags

function traverse(Id) {

   if(Id == null) return;
   stack.push(modelViewMatrix);
   modelViewMatrix = mult(modelViewMatrix, figure[Id].transform);
   figure[Id].render();
   if(figure[Id].child != null) traverse(figure[Id].child);
    modelViewMatrix = stack.pop();
   if(figure[Id].sibling != null) traverse(figure[Id].sibling);
}

function torso() {
    // ln properly set texture flags and uniforms
    sheepBodyText = true;
    sheepHeadText = false;
    grassText = false;
    fenceText = false;
    gl.uniform1f( gl.getUniformLocation(program, "uTextSheepBody"), sheepBodyText);
    gl.uniform1f( gl.getUniformLocation(program, "uTextSheepHead"), sheepHeadText);
    gl.uniform1f( gl.getUniformLocation(program, "uTextGrass"), grassText);
    gl.uniform1f( gl.getUniformLocation(program, "uTextFence"), fenceText);

    var ambientProduct = mult(lightAmbient, body_ambient);
    var diffuseProduct = mult(lightDiffuse, body_diffuse);
    gl.uniform4fv(gl.getUniformLocation(program, "uAmbientProduct"),
       flatten(ambientProduct));
    gl.uniform4fv(gl.getUniformLocation(program, "uDiffuseProduct"),
       flatten(diffuseProduct));

    instanceMatrix = mult(modelViewMatrix, translate(0.0, 0.5*torsoHeight, 0.0) );
    instanceMatrix = mult(instanceMatrix, scale( torsoWidth, torsoHeight, torsoWidth));
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix) );
    for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
}

function head() {
    // ln properly set texture flags and uniforms
    // ln N.B. the head needs some specifications:
    // ln only the front face does not have the bump, therefore:
    sheepBodyText = true;
    sheepHeadText = false;
    grassText = false;
    fenceText = false;
    gl.uniform1f( gl.getUniformLocation(program, "uTextSheepBody"), sheepBodyText);
    gl.uniform1f( gl.getUniformLocation(program, "uTextSheepHead"), sheepHeadText);
    gl.uniform1f( gl.getUniformLocation(program, "uTextGrass"), grassText);
    gl.uniform1f( gl.getUniformLocation(program, "uTextFence"), fenceText);

    var ambientProduct = mult(lightAmbient, head_ambient);
    var diffuseProduct = mult(lightDiffuse, head_diffuse);
    gl.uniform4fv(gl.getUniformLocation(program, "uAmbientProduct"),
       flatten(ambientProduct));
    gl.uniform4fv(gl.getUniformLocation(program, "uDiffuseProduct"),
       flatten(diffuseProduct));

    instanceMatrix = mult(modelViewMatrix, translate(0.0, 0.5 * headHeight, 0.0 ));
	instanceMatrix = mult(instanceMatrix, scale(headWidth, headHeight, headWidth) );
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix) );
    for(var i =0; i<6; i++) {
        // ln here, we switch to the light grey color texture for the front face
        if (i == 1){
            sheepBodyText = false;
            sheepHeadText = true;
            gl.uniform1f( gl.getUniformLocation(program, "uTextSheepBody"), sheepBodyText);
            gl.uniform1f( gl.getUniformLocation(program, "uTextSheepHead"), sheepHeadText);

        }
        // ln otherwise, use the bump texture as for the body
        else {
            sheepBodyText = true;
            sheepHeadText = false;
            gl.uniform1f( gl.getUniformLocation(program, "uTextSheepBody"), sheepBodyText);
            gl.uniform1f( gl.getUniformLocation(program, "uTextSheepHead"), sheepHeadText);
        }
        gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
    }
}

function leftUpperArm() {
    // ln properly set texture flags and uniforms
    sheepBodyText = true;
    sheepHeadText = false;
    grassText = false;
    fenceText = false;
    gl.uniform1f( gl.getUniformLocation(program, "uTextSheepBody"), sheepBodyText);
    gl.uniform1f( gl.getUniformLocation(program, "uTextSheepHead"), sheepHeadText);
    gl.uniform1f( gl.getUniformLocation(program, "uTextGrass"), grassText);
    gl.uniform1f( gl.getUniformLocation(program, "uTextFence"), fenceText);

    var ambientProduct = mult(lightAmbient, body_ambient);
    var diffuseProduct = mult(lightDiffuse, body_diffuse);
    gl.uniform4fv(gl.getUniformLocation(program, "uAmbientProduct"),
       flatten(ambientProduct));
    gl.uniform4fv(gl.getUniformLocation(program, "uDiffuseProduct"),
       flatten(diffuseProduct));

    instanceMatrix = mult(modelViewMatrix, translate(0.0, 0.5 * upperArmHeight, 0.0) );
	instanceMatrix = mult(instanceMatrix, scale(upperArmWidth, upperArmHeight, upperArmWidth) );
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix) );
    for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
}

function leftLowerArm() {
    // ln properly set texture flags and uniforms
    sheepBodyText = true;
    sheepHeadText = false;
    grassText = false;
    fenceText = false;
    gl.uniform1f( gl.getUniformLocation(program, "uTextSheepBody"), sheepBodyText);
    gl.uniform1f( gl.getUniformLocation(program, "uTextSheepHead"), sheepHeadText);
    gl.uniform1f( gl.getUniformLocation(program, "uTextGrass"), grassText);
    gl.uniform1f( gl.getUniformLocation(program, "uTextFence"), fenceText);

    var ambientProduct = mult(lightAmbient, body_ambient);
    var diffuseProduct = mult(lightDiffuse, body_diffuse);
    gl.uniform4fv(gl.getUniformLocation(program, "uAmbientProduct"),
       flatten(ambientProduct));
    gl.uniform4fv(gl.getUniformLocation(program, "uDiffuseProduct"),
       flatten(diffuseProduct));

    instanceMatrix = mult(modelViewMatrix, translate(0.0, 0.5 * lowerArmHeight, 0.0) );
	instanceMatrix = mult(instanceMatrix, scale(lowerArmWidth, lowerArmHeight, lowerArmWidth) );
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix) );
    for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
}

function rightUpperArm() {
    // ln properly set texture flags and uniforms
    sheepBodyText = true;
    sheepHeadText = false;
    grassText = false;
    fenceText = false;
    gl.uniform1f( gl.getUniformLocation(program, "uTextSheepBody"), sheepBodyText);
    gl.uniform1f( gl.getUniformLocation(program, "uTextSheepHead"), sheepHeadText);
    gl.uniform1f( gl.getUniformLocation(program, "uTextGrass"), grassText);
    gl.uniform1f( gl.getUniformLocation(program, "uTextFence"), fenceText);

    var ambientProduct = mult(lightAmbient, body_ambient);
    var diffuseProduct = mult(lightDiffuse, body_diffuse);
    gl.uniform4fv(gl.getUniformLocation(program, "uAmbientProduct"),
       flatten(ambientProduct));
    gl.uniform4fv(gl.getUniformLocation(program, "uDiffuseProduct"),
       flatten(diffuseProduct));

    instanceMatrix = mult(modelViewMatrix, translate(0.0, 0.5 * upperArmHeight, 0.0) );
	instanceMatrix = mult(instanceMatrix, scale(upperArmWidth, upperArmHeight, upperArmWidth) );
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix) );
    for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
}

function rightLowerArm() {
    // ln properly set texture flags and uniforms
    sheepBodyText = true;
    sheepHeadText = false;
    grassText = false;
    fenceText = false;
    gl.uniform1f( gl.getUniformLocation(program, "uTextSheepBody"), sheepBodyText);
    gl.uniform1f( gl.getUniformLocation(program, "uTextSheepHead"), sheepHeadText);
    gl.uniform1f( gl.getUniformLocation(program, "uTextGrass"), grassText);
    gl.uniform1f( gl.getUniformLocation(program, "uTextFence"), fenceText);

    var ambientProduct = mult(lightAmbient, body_ambient);
    var diffuseProduct = mult(lightDiffuse, body_diffuse);
    gl.uniform4fv(gl.getUniformLocation(program, "uAmbientProduct"),
       flatten(ambientProduct));
    gl.uniform4fv(gl.getUniformLocation(program, "uDiffuseProduct"),
       flatten(diffuseProduct));

    instanceMatrix = mult(modelViewMatrix, translate(0.0, 0.5 * lowerArmHeight, 0.0) );
	instanceMatrix = mult(instanceMatrix, scale(lowerArmWidth, lowerArmHeight, lowerArmWidth) );
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix) );
    for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
}

function  leftUpperLeg() {
    // ln properly set texture flags and uniforms
    sheepBodyText = true;
    sheepHeadText = false;
    grassText = false;
    fenceText = false;
    gl.uniform1f( gl.getUniformLocation(program, "uTextSheepBody"), sheepBodyText);
    gl.uniform1f( gl.getUniformLocation(program, "uTextSheepHead"), sheepHeadText);
    gl.uniform1f( gl.getUniformLocation(program, "uTextGrass"), grassText);
    gl.uniform1f( gl.getUniformLocation(program, "uTextFence"), fenceText);

    var ambientProduct = mult(lightAmbient, body_ambient);
    var diffuseProduct = mult(lightDiffuse, body_diffuse);
    gl.uniform4fv(gl.getUniformLocation(program, "uAmbientProduct"),
       flatten(ambientProduct));
    gl.uniform4fv(gl.getUniformLocation(program, "uDiffuseProduct"),
       flatten(diffuseProduct));

    instanceMatrix = mult(modelViewMatrix, translate(0.0, 0.5 * upperLegHeight, 0.0) );
	instanceMatrix = mult(instanceMatrix, scale(upperLegWidth, upperLegHeight, upperLegWidth) );
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix) );
    for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
}

function leftLowerLeg() {
    // ln properly set texture flags and uniforms
    sheepBodyText = true;
    sheepHeadText = false;
    grassText = false;
    fenceText = false;
    gl.uniform1f( gl.getUniformLocation(program, "uTextSheepBody"), sheepBodyText);
    gl.uniform1f( gl.getUniformLocation(program, "uTextSheepHead"), sheepHeadText);
    gl.uniform1f( gl.getUniformLocation(program, "uTextGrass"), grassText);
    gl.uniform1f( gl.getUniformLocation(program, "uTextFence"), fenceText);

    var ambientProduct = mult(lightAmbient, body_ambient);
    var diffuseProduct = mult(lightDiffuse, body_diffuse);
    gl.uniform4fv(gl.getUniformLocation(program, "uAmbientProduct"),
       flatten(ambientProduct));
    gl.uniform4fv(gl.getUniformLocation(program, "uDiffuseProduct"),
       flatten(diffuseProduct));

    instanceMatrix = mult(modelViewMatrix, translate( 0.0, 0.5 * lowerLegHeight, 0.0) );
	instanceMatrix = mult(instanceMatrix, scale(lowerLegWidth, lowerLegHeight, lowerLegWidth) );
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix) );
    for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
}

function rightUpperLeg() {
    // ln properly set texture flags and uniforms
    sheepBodyText = true;
    sheepHeadText = false;
    grassText = false;
    fenceText = false;
    gl.uniform1f( gl.getUniformLocation(program, "uTextSheepBody"), sheepBodyText);
    gl.uniform1f( gl.getUniformLocation(program, "uTextSheepHead"), sheepHeadText);
    gl.uniform1f( gl.getUniformLocation(program, "uTextGrass"), grassText);
    gl.uniform1f( gl.getUniformLocation(program, "uTextFence"), fenceText);

    var ambientProduct = mult(lightAmbient, body_ambient);
    var diffuseProduct = mult(lightDiffuse, body_diffuse);
    gl.uniform4fv(gl.getUniformLocation(program, "uAmbientProduct"),
       flatten(ambientProduct));
    gl.uniform4fv(gl.getUniformLocation(program, "uDiffuseProduct"),
       flatten(diffuseProduct));

    instanceMatrix = mult(modelViewMatrix, translate(0.0, 0.5 * upperLegHeight, 0.0) );
	instanceMatrix = mult(instanceMatrix, scale(upperLegWidth, upperLegHeight, upperLegWidth) );
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix) );
    for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
}

function rightLowerLeg() {
    // ln properly set texture flags and uniforms
    sheepBodyText = true;
    sheepHeadText = false;
    grassText = false;
    fenceText = false;
    gl.uniform1f( gl.getUniformLocation(program, "uTextSheepBody"), sheepBodyText);
    gl.uniform1f( gl.getUniformLocation(program, "uTextSheepHead"), sheepHeadText);
    gl.uniform1f( gl.getUniformLocation(program, "uTextGrass"), grassText);
    gl.uniform1f( gl.getUniformLocation(program, "uTextFence"), fenceText);
    
    var ambientProduct = mult(lightAmbient, body_ambient);
    var diffuseProduct = mult(lightDiffuse, body_diffuse);
    gl.uniform4fv(gl.getUniformLocation(program, "uAmbientProduct"),
       flatten(ambientProduct));
    gl.uniform4fv(gl.getUniformLocation(program, "uDiffuseProduct"),
       flatten(diffuseProduct));

    instanceMatrix = mult(modelViewMatrix, translate(0.0, 0.5 * lowerLegHeight, 0.0) );
	instanceMatrix = mult(instanceMatrix, scale(lowerLegWidth, lowerLegHeight, lowerLegWidth) )
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix) );
    for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
}

// ln tail render function
function tail() {
    // ln properly set texture flags and uniforms
    sheepBodyText = true;
    sheepHeadText = false;
    grassText = false;
    fenceText = false;
    gl.uniform1f( gl.getUniformLocation(program, "uTextSheepBody"), sheepBodyText);
    gl.uniform1f( gl.getUniformLocation(program, "uTextSheepHead"), sheepHeadText);
    gl.uniform1f( gl.getUniformLocation(program, "uTextGrass"), grassText);
    gl.uniform1f( gl.getUniformLocation(program, "uTextFence"), fenceText);
  
    var ambientProduct = mult(lightAmbient, body_ambient);
    var diffuseProduct = mult(lightDiffuse, body_diffuse);
    gl.uniform4fv(gl.getUniformLocation(program, "uAmbientProduct"),
       flatten(ambientProduct));
    gl.uniform4fv(gl.getUniformLocation(program, "uDiffuseProduct"),
       flatten(diffuseProduct));

    instanceMatrix = mult(modelViewMatrix, translate(0.0, 0.5 * tailHeight, 0.0) );
	instanceMatrix = mult(instanceMatrix, scale(tailWidth, tailHeight, tailWidth) );
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix) );
    for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
}

// ln grass field render function
function grass(){
    // ln properly set texture flags and uniforms
    sheepBodyText = false;
    sheepHeadText = false;
    grassText = true;
    fenceText = false;
    gl.uniform1f( gl.getUniformLocation(program, "uTextSheepBody"), sheepBodyText);
    gl.uniform1f( gl.getUniformLocation(program, "uTextSheepHead"), sheepHeadText);
    gl.uniform1f( gl.getUniformLocation(program, "uTextGrass"), grassText);
    gl.uniform1f( gl.getUniformLocation(program, "uTextFence"), fenceText);

    var ambientProduct = mult(lightAmbient, grass_ambient);
    var diffuseProduct = mult(lightDiffuse, grass_diffuse);
    gl.uniform4fv(gl.getUniformLocation(program, "uAmbientProduct"),
       flatten(ambientProduct));
    gl.uniform4fv(gl.getUniformLocation(program, "uDiffuseProduct"),
       flatten(diffuseProduct));

    instanceMatrix = mult(modelViewMatrix, translate(0.0, 0.5 * grassHeight, 0.0) );
	instanceMatrix = mult(instanceMatrix, scale(grassWidth, grassHeight, grassWidth) );
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix) );
    for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
}

// ln fence render function
function fence(){
    // ln properly set texture flags and uniforms
    sheepBodyText = false;
    sheepHeadText = false;
    grassText = false;
    fenceText = true;
    gl.uniform1f( gl.getUniformLocation(program, "uTextSheepBody"), sheepBodyText);
    gl.uniform1f( gl.getUniformLocation(program, "uTextSheepHead"), sheepHeadText);
    gl.uniform1f( gl.getUniformLocation(program, "uTextGrass"), grassText);
    gl.uniform1f( gl.getUniformLocation(program, "uTextFence"), fenceText);

    var ambientProduct = mult(lightAmbient, fence_ambient);
    var diffuseProduct = mult(lightDiffuse, fence_diffuse);
    gl.uniform4fv(gl.getUniformLocation(program, "uAmbientProduct"),
       flatten(ambientProduct));
    gl.uniform4fv(gl.getUniformLocation(program, "uDiffuseProduct"),
       flatten(diffuseProduct));

    instanceMatrix = mult(modelViewMatrix, translate(0.0, 0.5 * fenceHeight, 0.0) );
	instanceMatrix = mult(instanceMatrix, scale(fenceWidth, fenceHeight, fenceWidth) );
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix) );
    for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
}

// ln eye render function
function eye(){
    // ln properly set texture flags and uniforms
    sheepBodyText = false;
    sheepHeadText = false;
    grassText = false;
    fenceText = false;
    gl.uniform1f( gl.getUniformLocation(program, "uTextSheepBody"), sheepBodyText);
    gl.uniform1f( gl.getUniformLocation(program, "uTextSheepHead"), sheepHeadText);
    gl.uniform1f( gl.getUniformLocation(program, "uTextGrass"), grassText);
    gl.uniform1f( gl.getUniformLocation(program, "uTextFence"), fenceText);

    instanceMatrix = mult(modelViewMatrix, translate(0.0, 0.5 * eyeHeight, 0.0) );
	instanceMatrix = mult(instanceMatrix, scale(eyeWidth, eyeHeight, eyeWidth) );
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix) );
    for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
}

function quad(a, b, c, d) {
    // ln standard 3-dimensional normal associated to each vertex
    var t1 = subtract(vertices[b], vertices[a]);
    var t2 = subtract(vertices[c], vertices[b]);
    var normal = cross(t1, t2);
    normal = vec3(normal);
  
    // ln tangent vector is orthogonal to the normal
    // ln I can therefore imagine the tangent as one segment of the quad
    var tangent = vec3(t1[0], t1[1], t1[2]);

    // ln remember to add texture and tangent coordinates!
    
    pointsArray.push(vertices[a]);
    texCoordsArray.push(texCoord[0]);
    normalsArray.push(normal);
    tangentsArray.push(tangent);

    pointsArray.push(vertices[b]);
    texCoordsArray.push(texCoord[1]);
    normalsArray.push(normal);
    tangentsArray.push(tangent);

    pointsArray.push(vertices[c]);
    texCoordsArray.push(texCoord[2]);
    normalsArray.push(normal);
    tangentsArray.push(tangent);

    pointsArray.push(vertices[d]);
    texCoordsArray.push(texCoord[3]);
    normalsArray.push(normal);
    tangentsArray.push(tangent);
}

function cube()
{
    quad( 1, 0, 3, 2 );
    quad( 2, 3, 7, 6 );
    quad( 3, 0, 4, 7 );
    quad( 6, 5, 1, 2 );
    quad( 4, 5, 6, 7 );
    quad( 5, 4, 0, 1 );
}


// ln practical function to change text on a button
function changeButtonText(id, text)
{
    var elem = document.getElementById(id);
    elem.value = text;
}

var nMatrix;

window.onload = function init() {

    canvas = document.getElementById( "gl-canvas" );

    gl = canvas.getContext('webgl2');
    if (!gl) { alert( "WebGL 2.0 isn't available" ); }

    gl.viewport( 0, 0, canvas.width, canvas.height );
    gl.clearColor( 0.0, 1.0, 1.0, 1.0 );

    gl.enable(gl.DEPTH_TEST);

    //
    //  Load shaders and initialize attribute buffers
    //
    program = initShaders( gl, "vertex-shader", "fragment-shader");

    gl.useProgram( program);

    instanceMatrix = mat4();

    projectionMatrix = ortho(left, right, bottom, topp, near, far);
    modelViewMatrix = mat4();


    gl.uniformMatrix4fv(gl.getUniformLocation( program, "modelViewMatrix"), false, flatten(modelViewMatrix)  );
    gl.uniformMatrix4fv( gl.getUniformLocation( program, "projectionMatrix"), false, flatten(projectionMatrix)  );

    modelViewMatrixLoc = gl.getUniformLocation(program, "modelViewMatrix")
    projectionMatrixLoc = gl.getUniformLocation(program, "uProjectionMatrix");
    nMatrixLoc = gl.getUniformLocation(program, "uNormalMatrix");

    cube();

    vBuffer = gl.createBuffer();

    gl.bindBuffer( gl.ARRAY_BUFFER, vBuffer );
    gl.bufferData(gl.ARRAY_BUFFER, flatten(pointsArray), gl.STATIC_DRAW);

    var positionLoc = gl.getAttribLocation( program, "aPosition" );
    gl.vertexAttribPointer( positionLoc, 4, gl.FLOAT, false, 0, 0 );
    gl.enableVertexAttribArray( positionLoc );

    // ln normalsArray
    var nBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, nBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, flatten(normalsArray), gl.STATIC_DRAW);

    var normalLoc = gl.getAttribLocation(program, "aNormal");
    gl.vertexAttribPointer(normalLoc, 3, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(normalLoc);

    // ln texCoordsArray
    tBuffer = gl.createBuffer();
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

    // ln creating all the textures I have ...
    sheepBodyImage = createBumpTexture();
    sheepHeadImage = createColorTexture(0.65, 0.65, 0.65);
    grassImage = document.getElementById("grassImage");
    fenceImage = document.getElementById("fenceImage");

    configureTexture();
    
    // ln ... and remember to active and bind each of them
    gl.activeTexture(gl.TEXTURE0);
    gl.bindTexture(gl.TEXTURE_2D, textureBodySheep);
    gl.uniform1i(gl.getUniformLocation(program, "uTextureMapSheepBody"), 0);

    gl.activeTexture(gl.TEXTURE1);
    gl.bindTexture(gl.TEXTURE_2D, textureHeadSheep);
    gl.uniform1i(gl.getUniformLocation(program, "uTextureMapSheepHead"), 1);

    gl.activeTexture(gl.TEXTURE2);
    gl.bindTexture(gl.TEXTURE_2D, textureGrass);
    gl.uniform1i(gl.getUniformLocation(program, "uTextureMapGrass"), 2);

    gl.activeTexture(gl.TEXTURE3);
    gl.bindTexture(gl.TEXTURE_2D, textureFence);
    gl.uniform1i(gl.getUniformLocation(program, "uTextureMapFence"), 3);

    document.getElementById("ButtonAnimation").onclick = function(){
        animation = !animation;
        if (animation){
          changeButtonText("ButtonAnimation", "Reset Scene");
          animate();
        }
        else{
          changeButtonText("ButtonAnimation", "Start Animation");
          reset();
        }
      };

    document.getElementById("depthSlider").onchange = function(event) {
       far = event.target.value/2;
       near = -event.target.value/2;
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
    document.getElementById("heightSlider").onchange = function(event) {
        topp = event.target.value/2;
        bottom = -event.target.value/2;
    };
    document.getElementById("widthSlider").onchange = function(event) {
        right = event.target.value/2;
        left = -event.target.value/2;
    };

    gl.uniform4fv(gl.getUniformLocation(program, "uLightPosition1"),
       flatten(lightPosition1));
    gl.uniform4fv(gl.getUniformLocation(program, "uLightPosition2"),
       flatten(lightPosition2));

    for(i=0; i<numNodes; i++) initNodes(i);

    render();
}

// ln variable to switch between the motion of the legs
// ln in the two directions
var alternateLeg = true;
// ln rotation of the head
var alternateHead = true;

// ln here the stuff for the animation is done
var animate = function (){
    // ln the first part is to reach the fence at x = 0
    // ln the sheep will arrive close to it, at - 4.0
    if (torso_x < - 4.0){

        // ln the motion of the torso is just an incremental translation along the x-axis
        torso_x += 0.015;

        // ln now the tricky part to move the legs:
        // ln I have to modify the theta array setting the angles according to the ids
        if (alternateLeg){
   
            if (theta[leftUpperArmId] < -190){
                alternateLeg = false;
            }

            theta[leftUpperArmId] += -1;
            theta[rightUpperLegId] += -1;
            theta[leftLowerArmId] += 2;
            theta[rightLowerLegId] += 2;

            theta[rightUpperArmId] += 1;
            theta[leftUpperLegId] += 1;
            theta[rightLowerArmId] = 0;
            theta[leftLowerLegId] = 0;
            
        }
        else {
            if (theta[leftUpperArmId] > -170){
                alternateLeg = true;
            }
            theta[leftUpperArmId] += 1;
            theta[rightUpperLegId] += 1;
            theta[leftLowerArmId] = 0;
            theta[rightLowerLegId] = 0;

            theta[rightUpperArmId] += -1;
            theta[leftUpperLegId] += -1;
            theta[rightLowerArmId] += 2;
            theta[leftLowerLegId] += 2;
            
        }

        // ln rotating also the head and the tail at the same rythm
        if (alternateHead){
            if (theta[head2Id] > 10){
                alternateHead = false;
            }

            theta[head2Id] += 1;
            theta[tailId] += 2;
        }
        else{
            if (theta[head2Id] < -10){
                alternateHead = true;
            }

            theta[head2Id] += -1;
            theta[tailId] += -2;
        } 
    
    }
    // ln now I have to jump the fence and reach the point x = 3
    else if (torso_x < 5.0){
        // ln it is convenient to divide the jump in two phases:
        // ln 1) the ascending step
        if (torso_x < 0.5){
            // ln getting ready for the jump,
            // ln namely I have to rotate the torso a bit
            if (theta[torsoId] > -20){
                theta[torsoId] += -0.5;

                // ln restoring head and tail orientation
                theta[head2Id] = 0;
                theta[tailId] = -120;
            }
            else{

                // ln leave the surface until I reach the maximum height at the fence level

                torso_x += 0.09;
                torso_y += 0.15;
                theta[leftLowerArmId] = 45;
                theta[rightLowerLegId] = 45;
                theta[rightLowerArmId] = 45;
                theta[leftLowerLegId] = 45;
            }
        }

        // ln 2) the descending step
        else{

            torso_x += 0.09;

            // ln getting ready for the landing
            if (theta[torsoId] != 0){
                theta[torsoId] += 0.5;
            }
            
            torso_y += -0.15;
            theta[leftLowerArmId] = 45;
            theta[rightLowerLegId] = 45;
            theta[rightLowerArmId] = 45;
            theta[leftLowerLegId] = 45;
        }

    }

    // ln finally, let's land on the grass and do some others steps
    else if (torso_x < 8){

        torso_x += 0.015;
        torso_y = -6;

        // ln now the tricky part to move the legs:
        // ln I have to modify the theta array setting the angles according to the ids
        if (alternateLeg){
   
            if (theta[leftUpperArmId] < -190){
                alternateLeg = false;
            }

            theta[leftUpperArmId] += -1;
            theta[rightUpperLegId] += -1;
            theta[leftLowerArmId] += 2;
            theta[rightLowerLegId] += 2;

            theta[rightUpperArmId] += 1;
            theta[leftUpperLegId] += 1;
            theta[rightLowerArmId] = 0;
            theta[leftLowerLegId] = 0;
            
        }
        else {
            if (theta[leftUpperArmId] > -170){
                alternateLeg = true;
            }
            theta[leftUpperArmId] += 1;
            theta[rightUpperLegId] += 1;
            theta[leftLowerArmId] = 0;
            theta[rightLowerLegId] = 0;

            theta[rightUpperArmId] += -1;
            theta[leftUpperLegId] += -1;
            theta[rightLowerArmId] += 2;
            theta[leftLowerLegId] += 2;
            
        }

        // ln rotating also the head
        if (alternateHead){
            if (theta[head2Id] > 10){
                alternateHead = false;
            }

            theta[head2Id] += 1;
            theta[tailId] += 2;
        }
        else{
            if (theta[head2Id] < -10){
                alternateHead = true;
            }

            theta[head2Id] += -1;
            theta[tailId] += -2;
        } 

    }
    
    // ln rendering all the parts interactively
    for(i=0; i<numNodes; i++) initNodes(i);

    // ln this is crucial to render the animation
    requestAnimationFrame(animate);
}

// ln simple function to reset the scene with a reload of the window
function reset(){
    window.location.reload();
}

var render = function() {

        gl.clear( gl.COLOR_BUFFER_BIT | gl.DEPTH_BIT);

        // ln adding viwerPos and lookAt function
        viewerPos = vec3(radius*Math.sin(phi),
        radius*Math.sin(thetaVal),
        radius*Math.cos(phi));
    
        modelViewMatrix = lookAt(viewerPos, at , up);
        projectionMatrix = ortho(left, right, bottom, topp, near, far); 
        gl.uniformMatrix4fv( gl.getUniformLocation( program, "projectionMatrix"), false, flatten(projectionMatrix)  );

        nMatrix = normalMatrix(modelViewMatrix, true );
        gl.uniformMatrix3fv(nMatrixLoc, false, flatten(nMatrix));

        traverse(torsoId);

        // ln traversing grass tree (even if it is a single object)
        traverse(grassId);
        
        // ln traversing fence tree (even if it is a single object)
        traverse(fenceId1);
        
        requestAnimationFrame(render);
}

