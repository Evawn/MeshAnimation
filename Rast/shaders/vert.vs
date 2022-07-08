#version 330

uniform mat4 mM;  // Model matrix
uniform mat4 mV;  // View matrix
uniform mat4 mP;  // Projection matrix

uniform mat4 boneTransforms[50]; 
//uniform mat4 theBoneTransform;

uniform bool hasBones;

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec4 boneIds;
layout(location = 3) in vec4 boneWeights;

out vec3 vPosition;  // vertex position in eye space
out vec3 vNormal;

void main() {
    if(hasBones) {
        mat4 boneT0 = boneTransforms[int(boneIds[0])];
        mat4 boneT1 = boneTransforms[int(boneIds[1])];
        mat4 boneT2 = boneTransforms[int(boneIds[2])];
        mat4 boneT3 = boneTransforms[int(boneIds[3])];

        float w0 = boneWeights[0];
        float w1 = boneWeights[1];
        float w2 = boneWeights[2];
        float w3 = boneWeights[3];

        mat4 transMatrix = (w0 * boneT0) + (w1 * boneT1) + (w2 * boneT2) + (w3 * boneT3);
        vec4 transformed = transMatrix * vec4(position, 1.0);

        vPosition = (mV * mM * transformed).xyz;
        vNormal = (mV * mM * transMatrix * vec4(normal, 0.0)).xyz;
        gl_Position = mP * vec4(vPosition, 1.0);
    } else {
        vPosition = (mV * mM * vec4(position, 1.0)).xyz;
        vNormal = (mV * mM * vec4(normal, 0.0)).xyz;
        gl_Position = mP * vec4(vPosition, 1.0);
    }

}
