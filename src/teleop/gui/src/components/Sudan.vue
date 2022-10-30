<template>
    <div>
        <h3>Sudan III Drop</h3>
        <button id="sudan-button">Start Site {{site}} Test</button>
    </div>
</template>

<script>
export default {
    data() {
        return {
            
        }
    },

    props: {
        site: {
            type: String,
            required: true,
        }
    },
}
</script>

<style scoped>

</style>
