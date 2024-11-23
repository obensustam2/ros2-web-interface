<template>
    <div>
        <ul>
            <li 
                v-for="(cake, index) in cakes" 
                :key="index" 
                @click="handleCakeClick(cake)">
                <p>{{ cake.name }}</p>
                <p v-show="cake.show">{{ cake.taste }}</p>
            </li>
        </ul>
    </div>
</template>

<script>

    export default {
        data() {
            return {
                cakes: [
                    { name: 'Chocolate', taste: 'super', show: false },
                    { name: 'Bananacake', taste: 'super+', show: false },
                    { name: 'Muffins', taste: 'super++', show: false },
                    { name: 'Donuts', taste: 'perfect', show: false },
                    { name: 'Healthy Cake', taste: 'perfect+', show: false }
                ]
            }
        },
        
        methods: {
            handleCakeClick(cake) {
                // Toggle the `show` property
                cake.show = !cake.show;
            }
        }
    }
</script>

<style scoped>
    ul {
        display: flex;
        max-width: 1000px;
        margin: 20px auto;
        padding: 10px;
        box-sizing: border-box;
    }

    li {
        list-style: none;
        margin: 10px;
        padding: 20px;
        text-align: center;
        border: 1px solid #229966;
        flex-basis: 200px;
        cursor: pointer;
    }
</style>
